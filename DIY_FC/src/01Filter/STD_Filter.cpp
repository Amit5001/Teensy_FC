#include "STD_Filter.h"

STD_Filter::STD_Filter(Measurement_t* meas_data, float sample_hz) {
    this->_meas_data = meas_data;
    this->_sample_hz = sample_hz;
}

void STD_Filter::Acc_LPF() {
    _meas_data->acc_LPF.y = ALPHA_ACC_LPF * _meas_data->acc.y + (1 - ALPHA_ACC_LPF) * _meas_data->acc_LPF.y;
    _meas_data->acc_LPF.x = ALPHA_ACC_LPF * _meas_data->acc.x + (1 - ALPHA_ACC_LPF) * _meas_data->acc_LPF.x;
    _meas_data->acc_LPF.z = ALPHA_ACC_LPF * _meas_data->acc.z + (1 - ALPHA_ACC_LPF) * _meas_data->acc_LPF.z;
}

void STD_Filter::Gyro_LPF() {
    _meas_data->gyro_LPF.x = ALPHA_GYRO_LPF * _meas_data->gyroDEG.x + (1 - ALPHA_GYRO_LPF) * _meas_data->gyro_LPF.x;
    _meas_data->gyro_LPF.y = ALPHA_GYRO_LPF * _meas_data->gyroDEG.y + (1 - ALPHA_GYRO_LPF) * _meas_data->gyro_LPF.y;
    _meas_data->gyro_LPF.z = ALPHA_GYRO_LPF * _meas_data->gyroDEG.z + (1 - ALPHA_GYRO_LPF) * _meas_data->gyro_LPF.z;
}

void STD_Filter::Gyro_HPF() {
    _meas_data->gyro_HPF.x = ALPHA_HPF * (_meas_data->gyro_HPF.x + _meas_data->gyroRAD.x - _gyroPrev.x);
    _meas_data->gyro_HPF.y = ALPHA_HPF * (_meas_data->gyro_HPF.y + _meas_data->gyroRAD.y - _gyroPrev.y);
    _meas_data->gyro_HPF.z = ALPHA_HPF * (_meas_data->gyro_HPF.z + _meas_data->gyroRAD.z - _gyroPrev.z);

    _gyroPrev.x = _meas_data->gyroRAD.x;
    _gyroPrev.y = _meas_data->gyroRAD.y;
    _gyroPrev.z = _meas_data->gyroRAD.z;
}

void STD_Filter::A_filt() {
    // Acc_LPF();
    // Gyro_LPF();
    Gyro_HPF();
}