#!/usr/bin/env python


from dynamic_reconfigure.parameter_generator_catkin import *

def add_base_params(gen, prefix):
  #             Name                                               Type    Level Description                  Default    Min     Max
  gen.add(str(prefix) + "depth_gain",                              int_t,    1,  "Gain",                      16,        16,     248)
  gen.add(str(prefix) + "depth_enable_auto_exposure",              bool_t,   2,  "Enable Auto Exposure",      True)
  preset_enum = gen.enum([gen.const("Custom",        int_t,  0,  "Custom"),
                          gen.const("Default",       int_t,  1,  "Default Preset"),
                          gen.const("Hand",          int_t,  2,  "Hand Gesture"),
                          gen.const("HighAccuracy",  int_t,  3,  "High Accuracy"),
                          gen.const("HighDensity",   int_t,  4,  "High Density"),
                          gen.const("MediumDensity", int_t,  5,  "Medium Density")], "D400 Visual Presets")
  gen.add(str(prefix) + "depth_visual_preset",                     int_t,    3,  "D400 Visual Presets", 0, 0, 5, edit_method=preset_enum)
  gen.add(str(prefix) + "depth_frames_queue_size",                 int_t,    4,  "Frames Queue Size",         16,        0,      32)
  gen.add(str(prefix) + "depth_error_polling_enabled",             bool_t,   5,  "Error Polling Enabled",     False)
  gen.add(str(prefix) + "depth_output_trigger_enabled",            bool_t,   6,  "Output Trigger Enabled",    False)
  gen.add(str(prefix) + "depth_units",                             double_t, 7,  "Depth Units",               0.001,     0.001,  0.001)
  gen.add(str(prefix) + "JSON_file_path",                          str_t,    8,  "JSON_file_path",            "")
  gen.add(str(prefix) + "enable_depth_to_disparity_filter",        bool_t,   9,  "Enable Depth to Disparity Filter",   False)
  gen.add(str(prefix) + "enable_spatial_filter",                   bool_t,   10, "Enable Spatial Filter",              False)
  gen.add(str(prefix) + "enable_temporal_filter",                  bool_t,   11, "Enable Temporal Filter",             False)
  gen.add(str(prefix) + "enable_disparity_to_depth_filter",        bool_t,   12, "Enable Disparity to Depth Filter",   False)
  gen.add(str(prefix) + "spatial_filter_magnitude",                double_t, 13, "Spatial Filter Magnitude",         2.0,    1.0,    5.0)
  gen.add(str(prefix) + "spatial_filter_smooth_alpha",             double_t, 14, "Spatial Filter Smooth Alpha",      0.5,    0.25,   1.0)
  gen.add(str(prefix) + "spatial_filter_smooth_delta",             double_t, 15, "Spatial Filter Smooth Delta",      20.0,   1.0,    50.0)
  spatial_filter_holes_fill_enum = gen.enum([gen.const("SpatialFillHoleDisabled", int_t,  0,  "Spatial - Fill Hole Disabled"),
                                             gen.const("2PxielRadius",            int_t,  1,  "2-Pixel Radius"),
                                             gen.const("4PxielRadius",            int_t,  2,  "4-Pixel Radius"),
                                             gen.const("8PxielRadius",            int_t,  3,  "8-Pixel Radius"),
                                             gen.const("16PxielRadius",           int_t,  4,  "16-Pixel Radius"),
                                             gen.const("Unlimited",               int_t,  5,  "Unlimited")], "Spatial Filter Holes Fill")
  gen.add(str(prefix) + "spatial_filter_holes_fill",               int_t,    16, "Spatial Filter Holes Filter",      0,    0,     5, edit_method=spatial_filter_holes_fill_enum)
  gen.add(str(prefix) + "temporal_filter_smooth_alpha",            double_t, 17, "Temporal Smooth Alpha",            0.4,    0.0,   1.0)
  gen.add(str(prefix) + "temporal_filter_smooth_delta",            double_t, 18, "Temporal Smooth Delta",            20.0,   1.0,   100.0)
  temporal_filter_holes_fill_enum = gen.enum([gen.const("TemporalFillHoleDisabled",        int_t,  0,  "Temporal - Fill Hole Disabled"),
                                              gen.const("ValidIn8Of8",                     int_t,  1,  "Valid In 8 Of 8"),
                                              gen.const("ValidIn2Oflast3",                 int_t,  2,  "Valid In 2 Of last3"),
                                              gen.const("ValidIn2Oflast4",                 int_t,  3,  "Valid In 2 Of last4"),
                                              gen.const("ValidIn2Of8",                     int_t,  4,  "Valid In 2 Of 8"),
                                              gen.const("ValidIn1Oflast2",                 int_t,  5,  "Valid In 1 Of last2"),
                                              gen.const("ValidIn1Oflast5",                 int_t,  6,  "Valid in 1 Of last5")], "Temporal Filter Holes Fill")
  gen.add(str(prefix) + "temporal_filter_holes_fill",              int_t,    19, "Temporal Filter Holes Fill",       3,       0,    6, edit_method=temporal_filter_holes_fill_enum)