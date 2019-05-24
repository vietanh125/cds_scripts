#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

int d = 80;
int rows = 160;
int cols = 320;
int square = 7;
int margin_roi = 40;
float time_threshold = 12.0;

int max(int a, int b) {
    return (a >= b) ? a : b;
}

int min(int a, int b) {
    return (a <= b) ? a : b;
}

void get_point(int img[rows][cols], 
            float roi, 
            int* x, 
            int *y, 
            bool *is_crossroad, 
            bool *is_crossroad_control,
            int flag, 
            int left_restriction, 
            int right_restriction, 
            bool has_road, 
            int road_property, 
            bool has_road_05, 
            int road_property_05, 
            int margin, 
            int mode) {
    int border = (square - 1) / 2;
    int i = rows * roi;
    int i_l = border;
    int i_r = cols -1 - border;
    bool turn_left = false;
    bool turn_right = false;
    int white;

    while (i_l < cols - border) {
        white = 0;
        for (int m = i - border; m <= i + border; m++)
            for (int n = i_l - border; n <= i_l + border; n++) {
                white += img[m][n];
            }
        if (white == (int) pow(square, 2)) {
            if (i_l <= margin)
                turn_left = true;
            if (i_l >= left_restriction + border)
                break;
        }
        i_l += border + 1;
    }

    while (i_r > i_l) {
        white = 0;
        for (int m = i - border; m <= i + border; m++)
            for (int n = i_r - border; n <= i_r + border; n++)
                white += img[m][n];
        if (white == (int) pow(square, 2)) {
            if (i_r >= cols - margin)
                turn_right = true;
            if (i_r <= right_restriction + 1 - border)
                break;
        }
        i_r -= (border + 1);
    }

    if (turn_left && turn_right && roi == (float) 0.6 && has_road_05 && road_property_05 == 2)
        *is_crossroad= true;
    if (turn_left && turn_right)
        *is_crossroad_control = true;

    if (left_restriction > 0)
        turn_left = false;
    if (right_restriction < 319)
        turn_right = false;

    if ((turn_left && turn_right && flag == 1) || (turn_right && !turn_left && flag != -1 && !has_road)) {
        while (img[i][i_r] == 1 && i >= 0)
            i--;
        *x = i_r;
        *y = i + 1; 
        return;
    }
    else if ((turn_left && turn_right && flag == -1) || (turn_left && !turn_right && flag != 1 && !has_road)) {
        while (img[i][i_l] == 1 && i >= 0)
            i--;
        *x = i_l;
        *y = i + 1;
        return;
    }
    switch (mode) {
        case 0:
            *x = (i_l + i_r) / 2;
            break;
        case -1:
            *x = max(0, min(i_l + d, cols));
            break;
        case 1:
            *x = max(0, min(i_r - d, cols));
            break;          
    }
    *y = i;
}

void get_point_left_and_right(int img[rows][cols], 
                        float roi, 
                        int* x, 
                        int *y, 
                        int flag, 
                        int left_restriction, 
                        int right_restriction, 
                        bool has_road, 
                        int road_property, 
                        float total_time, 
                        int margin, int mode) {
    int border = (square - 1) / 2;
    int i = rows * roi;
    int i_l = left_restriction + border;
    int i_r = right_restriction - border;
    int white;

    while (i_l < right_restriction + 1 - border) {
        white = 0;
        for (int m = i - border; m <= i + border; m++)
            for (int n = i_l - border; n <= i_l + border; n++) {
                white += img[m][n];
            }
        if (white == (int) pow(square, 2)) {
            break;
        }
        i_l += border + 1;
    }
    while (i_r > i_l) {
        white = 0;
        for (int m = i - border; m <= i + border; m++)
            for (int n = i_r - border; n <= i_r + border; n++)
                white += img[m][n];
        // printf("%d\n", white);
        if (white == (int) pow(square, 2)) {
            break;
        }
        i_r -= (border + 1);
    }
    // if (total_time < time_threshold)
    switch (mode) {
        case -1:
            *x = max(0, min(i_l + 50, cols));
            break;
        default:
            *x = max(0, min(i_r - 50, cols));
    }
    // else
    //     *x = max(0, min(i_r - 50, cols));
    *y = i;
    // printf("di thang\n");    
}

void check_future_road(int img[rows][cols], 
                    float roi, 
                    bool *has_road, 
                    int *road_property, 
                    int margin) {
    int border = (square - 1) / 2;
    int i = rows * roi;
    int i_l = border;
    int i_r = cols - border;
    bool turn_left = false;
    bool turn_right = false;
    int white;
    *has_road = false;
    while (i_l < cols - border) {
        white = 0;
        for (int m = i - border; m <= i + border; m++)
            for (int n = i_l - border; n <= i_l + border; n++) {
                white += img[m][n];
            }
        if (white == (int) pow(square, 2)) {
            if (i_l <= margin)
                turn_left = true;
            *has_road = true;
            break;
        }
        i_l += border + 1;
    }
    while (i_r > i_l) {
        white = 0;
        for (int m = i - border; m <= i + border; m++)
            for (int n = i_r - border; n <= i_r + border; n++)
                white += img[m][n];
        // printf("%d\n", white);
        if (white == (int) pow(square, 2)) {
            if (i_r >= cols - margin)
                turn_right = true;
            *has_road = true;
            break;
        }
        i_r -= (border + 1);
    }
    
    if (turn_left && !turn_right)
        *road_property = -1;
    else if (turn_right && !turn_left)
        *road_property = 1;
    else if (turn_left && turn_right)
        *road_property = 2;
    else
        *road_property = 0;
}

void get_center_point(int img[rows][cols], 
                    float roi, 
                    float future_roi, 
                    int flag, 
                    int *x, 
                    int *y, 
                    bool *is_crossroad,
                    bool *is_crossroad_control,
                    int left_restriction, 
                    int right_restriction, 
                    int mode) {
    // bool temp_bool = false;
    // int temp_int = 0;
    bool *has_road = malloc(sizeof(bool));
    int *road_property = malloc(sizeof(int));
    bool *has_road_05 = malloc(sizeof(bool));
    int *road_property_05 = malloc(sizeof(int));
    check_future_road(img, future_roi, has_road, road_property, 40);
    check_future_road(img, 0.5, has_road_05, road_property_05, 40);
    get_point(img, roi, x, y, is_crossroad, is_crossroad_control, flag, left_restriction, right_restriction, *has_road, *road_property, *has_road_05, *road_property_05, margin_roi, mode);
    while (img[*y][*x] == 0 && roi < 0.9) {
        *is_crossroad = false;
        roi += 0.05;
        get_point(img, roi, x, y, is_crossroad, is_crossroad_control, flag, left_restriction, right_restriction, *has_road, *road_property, *has_road_05, *road_property_05, margin_roi, mode);
    }
}

void get_center_point_left_and_right(int img[rows][cols], 
                                    float roi, 
                                    float future_roi, 
                                    int flag, 
                                    int *x, 
                                    int *y, 
                                    int left_restriction, 
                                    int right_restriction, 
                                    float total_time, 
                                    int mode) {
    // bool temp_bool = false;
    // int temp_int = 0;
    bool *has_road = malloc(sizeof(bool));
    int *road_property = malloc(sizeof(int));
    check_future_road(img, future_roi, has_road, road_property, 30);
    get_point_left_and_right(img, roi, x, y, flag, left_restriction, right_restriction, *has_road, *road_property, total_time, 10, mode);
    while (img[*y][*x] == 0 && roi < 0.9) {
        roi += 0.05;
        get_point_left_and_right(img, roi, x, y, flag, left_restriction, right_restriction, *has_road, *road_property, total_time, 10, mode);
    }
}

void set_time_threshold(float temp_var) {
    time_threshold = temp_var;
}

