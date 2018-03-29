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
