#include <limits>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "Local_View_Cell.hpp"
#include "Local_View_Network.hpp"
#include "Pose_Cell_Network.hpp"
#include "Experience_Map.hpp"
#include "Mapper.hpp"
using namespace cv;

int main(int argc, char **argv) {

    /* Streams for diagnostic output. */
    std::ofstream fout;
    fout.open("ratslamout.txt");
    std::string difference;
    std::string network_size;
    std::string match_index;
    std::string width;
    std::string height;
    std::string vrot;
    std::string vtrans;
    std::string injected_energy;
    std::string shiftx, shifty;
    std::string shift;
    std::string bx, by, bth;
    std::stringstream convert;

    double time_s = 0;
    double time_diff = 0;

    Local_View_Network local_view_network;
    Pose_Cell_Network pose_cell_network(fout);
    Experience_Map experience_map;
    Mapper mapper(pose_cell_network.get_network(), pose_cell_network.get_pc_network_dimension_xy(), pose_cell_network.get_pc_network_dimension_th());
    pose_cell_network.get_network();

    double PC_VT_INJECT_ENERGY = 0.15;
    double PC_VTRANS_SCALING = 5.0;
    double EXP_DELTA_PC_THRESHOLD = 2.0;

    // Initialize webcam device.
    VideoCapture webcam(0);

    // Check if the webcam was properly initialized.
    if(!webcam.isOpened()) {
        return -1;
    }

    const int ROI_IMAGE_WIDTH = 320;
    const int ROI_IMAGE_HEIGHT = 240;

    // Corner Detection Stuff
    //const int MAX_CORNERS = 100; // The maximum number of corners allowed to detect.

    Mat frame; // Frame to capture webcam input.
    Mat grayscale; // Frame to hold grayscale conversion.
    Mat roi_image; // Frame to hold region of interest.
    Rect roi(160, 105, ROI_IMAGE_WIDTH, ROI_IMAGE_HEIGHT); // Coordinates for region of interest.

    // Corner Detection Stuff
    //std::vector<cv::Point2f> corners;
    //CvScalar target_color[4] = { // in BGR order
	//	{{   0,   0, 255,   0 }},  // red
	//	{{   0,   0,   0,   0 }},  // green
	//	{{   0,   0,   0,   0 }},  // blue
	//	{{   0,   0,   0,   0 }}   // yellow
    //};

    double *prev_column_sum_array;
    double mindiff = 0.0;
    long double minavgdiff = 0.0;
    double energy = 0.0;

    for(;;) {
        // Read frame from webcam.
        webcam >> frame;

        // Convert webcam capture frame to grayscale for processing.
        cvtColor(frame, grayscale, CV_BGR2GRAY);

        // Corner Detection Stuff
        //goodFeaturesToTrack(grayscale, corners, MAX_CORNERS, 0.1, 5.0);
        //
        //for(int i = 0; i < MAX_CORNERS; i++) {
        //    int radius = grayscale.rows/25;
        //
        //    circle(frame,
		//			cvPoint((int)(corners[i].x + 0.5f),(int)(corners[i].y + 0.5f)),
		//			radius,
		//			target_color[0]);
        //}

        // Reduce grayscale frame to specific region of interest.
        roi_image = frame(roi);

        // Initializes the column sum array.
        std::vector<double> scanline_intensity_profile(roi_image.cols);

        if(prev_column_sum_array != NULL) {
            delete [] prev_column_sum_array;
        }

        prev_column_sum_array = new double(roi_image.cols);
        //double max_column_sum = 0.0;

        // Loop to calculate intensity sums for all of the columns in the region of interest frame.
        for(int column = 0; column < roi_image.cols; column++) {
            Mat column_values; // Matrix to hold all the pixel values of the current column.
            Scalar column_sum; // Current column sum value.

            column_values = roi_image.col(column); // Grab column pixels.
            column_sum = sum(column_values); // Calculate column sum.

            scanline_intensity_profile[column] = norm(column_sum);
        }

        // Draw a column sum intensity graph onto the region of interest frame.
        for(int column = 0; column < roi_image.cols - 1; column++) {
            Point p1(column, scanline_intensity_profile[column] / roi_image.rows);
            Point p2(column + 1, scanline_intensity_profile[column + 1] / roi_image.rows);

            line(roi_image, p1, p2, CV_RGB(255, 255, 255));
        }

        time_diff = static_cast<double>(clock()) / CLOCKS_PER_SEC - time_s;
        time_s += time_diff;

        fout << "time_diff: " << time_diff << std::endl;
        fout << "time_s: " << time_s << std::endl;

        local_view_network.set_previous_view(local_view_network.get_current_view());
        local_view_network.set_current_view(scanline_intensity_profile);

        // Calculate visual odometry estimate.
        double velocity_rotational = 0.0;
        double velocity_translational = 0.0;
        int best_matching_shift = 0;
        minavgdiff = local_view_network.estimate_velocity(
            best_matching_shift,
            velocity_rotational,
            velocity_translational
        );

        velocity_rotational *= time_diff;
        velocity_translational *= time_diff;
        double shift_x = velocity_translational * std::cos(velocity_rotational);
        double shift_y = velocity_translational * std::sin(velocity_rotational);

        int best_matching_cell_index = 0;
        mindiff = local_view_network.match_current_view(best_matching_cell_index);

        if(local_view_network.test_match_quality(mindiff)) {
            local_view_network.set_current_view_cell_index(best_matching_cell_index);
            //energy = PC_VT_INJECT_ENERGY; // * 1.0/30.0; // * (30.0 - exp(1.2 * local_view_network.get_current_view_cell().get_decay()));
            local_view_network.get_current_view_cell().set_connection_weight(pose_cell_network.get_pose_cell_activity(
                local_view_network.get_current_view_cell().pc_x,
                local_view_network.get_current_view_cell().pc_y,
                local_view_network.get_current_view_cell().pc_th));
            pose_cell_network.inject(
                local_view_network.get_current_view_cell().get_connection_weight(),
                local_view_network.get_current_view_cell().pc_x,
                local_view_network.get_current_view_cell().pc_y,
                local_view_network.get_current_view_cell().pc_th);
        } else {
            local_view_network.set_current_view_cell_index(local_view_network.create_cell(
                pose_cell_network.get_best_x(), pose_cell_network.get_best_y(), pose_cell_network.get_best_th()));
        }

        pose_cell_network.integrate_path(velocity_rotational, velocity_translational);
        pose_cell_network.iterate(fout);
        pose_cell_network.find_best();

        /*Experience *experience;
        double delta_pc;
        int new_exp;

        experience_map.integrate_position(velocity_translational, velocity_rotational);

        experience = experience_map.get_experience(experience_map.get_current_experience_index());
        delta_pc = pose_cell_network.get_delta_pc(experience->get_pc_x(), experience->get_pc_y(), experience->get_pc_th());

        if(local_view_network.get_current_view_cell().get_experience_count() == 0) {
            new_exp = experience_map.create_experience(pose_cell_network.get_best_x(), pose_cell_network.get_best_y(), pose_cell_network.get_best_th(), time_diff);
            experience_map.set_current_experience_index(new_exp);
            local_view_network.get_current_view_cell().add_experience(new_exp);
        } else if(delta_pc > EXP_DELTA_PC_THRESHOLD || local_view_network.get_current_view_cell_index() != local_view_network.get_previous_view_cell_index()) {
            // go through all the exps associated with the current view and find the one with the closest delta_pc

            int matched_exp_id = -1;
            int min_delta_id;
            double min_delta = std::numeric_limits<double>::max();
            double delta_pc;

            // find the closest experience in pose cell space
            for(int i = 0; i < local_view_network.get_current_view_cell().get_experience_count(); i++) {
                experience = experience_map.get_experience(local_view_network.get_current_view_cell().get_experience_index(i));
                delta_pc = pose_cell_network.get_delta_pc(experience->get_pc_x(), experience->get_pc_y(), experience->get_pc_th());

                if(delta_pc < min_delta) {
                    min_delta = delta_pc;
                    min_delta_id = local_view_network.get_current_view_cell().get_experience_index(i);
                }
            }

            // If an experience is closer than the threshold, create a link.
            if(min_delta < EXP_DELTA_PC_THRESHOLD) {
                matched_exp_id = min_delta_id;
                experience_map.create_transition(experience_map.get_current_experience_index(), matched_exp_id, time_diff);
            }

            if(experience_map.get_current_experience_index() != matched_exp_id) {
                if(matched_exp_id == -1) {
                    new_exp = experience_map.create_experience(pose_cell_network.get_best_x(), pose_cell_network.get_best_y(), pose_cell_network.get_best_th(), time_diff);
                        experience_map.set_current_experience_index(new_exp);
                    local_view_network.get_current_view_cell().add_experience(new_exp);
                }
                else {
                    experience_map.set_current_experience_index(matched_exp_id);
                }
            } else if(local_view_network.get_current_view_cell_index() == local_view_network.get_previous_view_cell_index()) {
                new_exp = experience_map.create_experience(pose_cell_network.get_best_x(), pose_cell_network.get_best_y(), pose_cell_network.get_best_th(), time_diff);
                experience_map.set_current_experience_index(new_exp);
                local_view_network.get_current_view_cell().add_experience(new_exp);
            }
        }

        experience_map.iterate();*/


        convert << local_view_network.get_network_size();
        network_size = convert.str();
        convert.str(std::string());

        convert << mindiff;
        difference = convert.str();
        convert.str(std::string());

        convert << ROI_IMAGE_WIDTH;
        width = convert.str();
        convert.str(std::string());

        convert << ROI_IMAGE_HEIGHT;
        height = convert.str();
        convert.str(std::string());

        convert << velocity_rotational;
        vrot = convert.str();
        convert.str(std::string());

        convert << velocity_translational;
        vtrans = convert.str();
        convert.str(std::string());

        convert << best_matching_shift;
        shift = convert.str();
        convert.str(std::string());

        convert << shift_x;
        shiftx = convert.str();
        convert.str(std::string());

        convert << shift_y;
        shifty = convert.str();
        convert.str(std::string());

        convert << energy;
        injected_energy = convert.str();
        convert.str(std::string());

        convert << best_matching_cell_index;
        match_index = convert.str();
        convert.str(std::string());

        convert << pose_cell_network.get_best_x();
        bx = convert.str();
        convert.str(std::string());

        convert << pose_cell_network.get_best_y();
        by = convert.str();
        convert.str(std::string());

        convert << pose_cell_network.get_best_th();
        bth = convert.str();
        convert.str(std::string());

        putText(roi_image, width + " x " + height, cvPoint(10, 15), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "network size: " + network_size, cvPoint(10, 25), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "mindiff: " + difference, cvPoint(10, 35), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "vrot: " + vrot, cvPoint(10, 45), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "vtrans: " + vtrans, cvPoint(10, 55), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "shift_th: " + shift, cvPoint(10, 65), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "shift_x: " + shiftx, cvPoint(10, 75), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "shift_y: " + shifty, cvPoint(10, 85), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "energy: " + injected_energy, cvPoint(10, 95), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));

        if(mindiff <= local_view_network.get_view_matching_threshold()) {
            putText(roi_image, "MATCH", cvPoint(265, 15), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        }

        putText(roi_image, match_index, cvPoint(275, 25), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "bx: " + bx, cvPoint(275, 35), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "by: " + by, cvPoint(275, 45), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));
        putText(roi_image, "bt: " + bth, cvPoint(275, 55), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(0,0,255));

        // Display the region of interest frame.
        imshow("Camera Capture", roi_image);
        mapper.drawNetwork();

        if(waitKey(30) >= 0) {
            break;
        }
    }

    webcam.release();

    return 0;
}
