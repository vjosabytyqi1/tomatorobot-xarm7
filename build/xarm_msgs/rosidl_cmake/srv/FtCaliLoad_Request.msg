# This format is suitable for the following services
#   - ft_sensor_cali_load

# iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])
float32[] datas

# whether to convert the paramster to the corresponding tcp load and set, default is false
bool association_setting_tcp_load   false
float32 m   0.325
float32 x   -17.0
float32 y   9.0
float32 z   11.8

