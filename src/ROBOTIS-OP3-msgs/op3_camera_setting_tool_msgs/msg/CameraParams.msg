# This represents the parameters of usb camera

uint32  brightness 							            # 0 - 255, step = 1, default: 128
uint32  contrast                            # 0 - 255, step = 1, default: 128
uint32  saturation                          # 0 - 255, step = 1, default: 128
uint32  sharpness                           # 0 - 255, step = 1, default: 128
uint32  gain                                # 0 - 255, step = 1, default: 0
bool    focus_automatic_continuous          # default: true
int32   focus_absolute                      # 0 - 250, step = 5, default: 0
uint32  auto_exposure                       # 0 - 3, default: 3
uint32  exposure_time_absolute              # 3 - 2047, step = 1, default: 250
bool    white_balance_automatic             # default: true
uint32  white_balance_temperature           # 2000 - 6500, step = 1, default: 4000
