#include "TLA2024.h"


TLA2024::TLA2024(): devI2C_(&Wire) {
    devI2C_->begin(21, 22);
}

TLA2024::TLA2024(TwoWire *dev): devI2C_(dev) {
}

bool TLA2024::begin() {

    reset();
    delay(10);

    uint16_t init = read(confReg_);
    Serial.printf("Done init. 0x%X\n", init);

    // make sure communication with device is working and that it is OK
    return (init == initConf_) ? true : false;
}

bool TLA2024::begin(uint8_t address){
    addr = address;
    begin();
}

uint16_t TLA2024::read(uint8_t mem_addr) {
    devI2C_->beginTransmission(addr);
    devI2C_->write(mem_addr);
    devI2C_->endTransmission(false);
//    delay(5);
    devI2C_->requestFrom(addr, (uint8_t)2);
    if (2 <= devI2C_->available()) {
        // bring in data
        data.packet[1] = devI2C_->read();
        data.packet[0] = devI2C_->read();
        uint16_t ret = data.value;
        data.value = 0;
        return ret;
    }

    return 0;
}

int TLA2024::write(uint16_t out_data) {
    int written = 0;
    // save conf
    savedConf_ = out_data;
    // put our out_data into the I2C data union so we can send MSB and LSB
    data.value = out_data;
    devI2C_->beginTransmission(addr);
    devI2C_->write(confReg_);
    written += devI2C_->write(data.packet[1]);
    written += devI2C_->write(data.packet[0]);
    devI2C_->endTransmission();
    data.value = 0;
    return written;
}

void TLA2024::reset() {
    write(initConf_);
}

void TLA2024::restore() {
    uint16_t restore_conf = savedConf_ & ~0x8000;
    write(restore_conf);
    // Serial.println(restore_conf, BIN);
}

float TLA2024::analogRead() {
    // this only needs to run when in single shot.
    if (currentMode_ == OP_SINGLE) {
        // write 1 to OS bit to start conv
        uint16_t current_conf = read(confReg_);
        current_conf |= 0x8000;
        write(current_conf);
        // OS bit will be 0 until conv is done.
        do {
            delay(5);
        } while ((read(confReg_) & 0x8000) == 0);
    }

    // get data from conv_reg
    uint16_t in_data = read(convReg_);

    // shift out unused bits
    in_data >>= 4;

    // get sign and mask accordingly
    if (in_data & (1 << 11)) {
        // 11th bit is sign bit. if its set, set bits 15-12
        in_data |= 0xF000;
    } else {
        // not set, clear bits 15-12
        in_data &= ~0xF000;
    }

    // now store it as a signed 16 bit int.
    int16_t ret = in_data;

    // default Full Scale Range is -2.048V to 2.047V.
    // our 12bit 2's complement goes from -2048 to 2047 :)
    // return ret /1000.0;

    // return raw adc data
    return ret;
}

float TLA2024::analogRead(uint8_t channel) {
    MultiplexerConfig muxCfg = (MultiplexerConfig)(channel - 4);
    setMuxConfig(muxCfg);
    return analogRead();
}

void TLA2024::setFullScaleRange(TLA2024::FullScaleRange range) {
    currentFSR_val_ = range;

    // bring in conf reg
    uint16_t conf = read(confReg_);

    // clear the PGA bits:
    conf &= ~0x0E00;

    // shift
    conf |= range << 9;
    write(conf);
}

/*
void TLA2024::setFSR(uint8_t gain) {
    // bring in conf reg
    uint16_t conf = read(confReg_);
    // clear the PGA bits:
    conf &= ~0x0E00;

    switch (gain) {
        case 60:
            // PGA bits already cleared
            break;

        case 40:
            // set bit 9
            conf |= 0x0200;
            break;

        case 20:
            // set bit 10
            conf |= 0x0400;
            break;

        case 10:
            // set bit 10-9
            conf |= 0x0600;
            break;

        case 5:
            // set bit 11
            conf |= 0x0800;
            break;

        case 2:
            // set bit 11-9
            conf |= 0x0E00;
            break;
    }
    write(conf);
}
*/

void TLA2024::setMuxConfig(MultiplexerConfig option) {
    // bring in conf reg
    uint16_t conf = read(confReg_);
    // clear MUX bits
    conf &= ~0x7000;

    // shift
    conf |= option << 12;
    write(conf);
}

/*
void TLA2024::setMux(uint8_t option) {
    // bring in conf reg
    uint16_t conf = read(confReg_);
    // clear MUX bits
    conf &= ~0x7000;

    switch (option) {
        case 1:
            // bits already cleared
            conf |= 0x4000;
            break;

        case 2:
//            // set bit 12
//            conf |= 0x1000;
            conf |= 0x5000;
            break;

        case 3:
//            // set bit 13
//            conf |= 0x2000;
            conf |= 0x6000;
            break;

        case 4:
//            // set bit 14
//            conf |= 0x4000;
            conf |= 0x7000;
            break;
        default:
            break;
    }
    write(conf);
}
*/

void TLA2024::setOperatingMode(OperatingMode mode) {
    // bring in conf reg
    currentMode_ = mode;
    uint16_t conf = read(confReg_);

    // clear MODE bit (8) (continous conv)
    conf &= ~(1 << 8);
    if (mode == OP_SINGLE) {
        // single shot
        conf |= (1 << 8);
    }
    write(conf);
}

void TLA2024::setDataRate(DataRate rate) {
    // bring in conf reg
    uint16_t conf = read(confReg_);

    // set bits 7:5
//    conf |= 0b111 << 5;
    conf |= rate << 5;
    write(conf);
}

float TLA2024::getCurrentFullRangeVoltage() {
    uint16_t shifted = 8192 >> currentFSR_val_;
//    Serial.println(shifted);

    // Special case
    if (currentFSR_val_ == 0) {
        shifted = 6144;
    }

    return (shifted*1.0f/1000);
}

float TLA2024::voltageRead(uint8_t channel) {
    float val = analogRead(channel);
    float fsrV = getCurrentFullRangeVoltage();
//    long converted = map((long)val, -2048, 2047, 0, fsrV*1000 );
    long converted = map((long)val, 0, 2047, 0, fsrV*1000 );
//    Serial.printf("fsrV = %.4f, mapped = %lu\n", fsrV, converted);

    float convF = converted*1.0f/1000;
    return convF;
}





