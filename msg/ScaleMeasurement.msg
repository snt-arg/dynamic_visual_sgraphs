# W2 (docs/evaluation/results/w2_scale_controller/criterion.md sec 3.1): one robust pedestrian-
# derived scale measurement, already divided by the W1 calibration constant c.
#
# CONVENTION, fixed by W1 (t0b_regate/criterion.md sec 1): scale is METRES PER MAP UNIT -- the
# same factor Map::ApplyScaledRotation takes. scale > 1 means the map is too small. After a
# perfect correction the next measurement must read 1.0, NOT scale^2 and NOT 1/scale.
std_msgs/Header header

float64 scale        # robust median of s_person / c over the measurement window
float64 sigma        # MAD-derived standard error of that median
float64 window_s     # length of the measurement window, seconds

uint32 n_detections   # admitted detections in the window
uint32 n_sub_windows  # populated 30 s sub-windows (must be >= K_sub for the window to be valid)
uint32 n_tracks       # distinct tracks contributing
bool valid            # all population gates met
