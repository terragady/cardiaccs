/* *************************************************************************
 * Copyright (C) Cardiaccs AS - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Written by Marek Gayer <marek.gayer@cardiaccs.com> and 
 * Magnus Reinsfelt Krogh <magnus.krogh@cardiaccs.com>, 2017 - 2019
 *
 * *************************************************************************/

#ifndef FT_HPP
#define FT_HPP


    class FT
    {
        static constexpr int clockSpeed = 100;

    public:
        enum class Status
        {
            // Values always only between 0 and INT8_MIN(-128)
            // Note: our enumeration numerical values can change in future code revisions, always rely only on field names or message not the numeric codes
            ok = 0,
            unknownError = -1,
            programmingError = -2,
            timeout = -3,
            invalidDeviceId = -4,
            fifoTimeout = -5,
            tooManyFrames = -6,
            fifoFailure = -7,
            fileOpenError = -8,
            i2CmDataNack = -10,
            i2CmAddressNack = -11,
            i2CmArbLost = -12,
            accScaleError = -13,
            adcScaleError = -14,
            testConnectionFailed = -15,
            adcAllZeros = -20,
            readFailure = -21,
            writeFailure = -22,
            writeToDeviceRegisterNotAcknowledged = -23,

            ft4222UnknownStatus = -99,
            ft4222InvalidHandle = -98, // ...
            ft4222DeviceNotFound,
            ft4222DeviceNotOpened,
            ft4222IoError,
            ft4222InsufficientResources,
            ft4222InvalidParameter,
            ft4222InvalidBaudRate,
            ft4222DeviceNotOpenedForErase,
            ft4222DeviceNotOpenedForWrite,
            ft4222FailedToWriteDevice,
            ft4222EepromReadFailed,
            ft4222EepromWriteFailed,
            ft4222EepromEraseFailed,
            ft4222EepromNotPresent,
            ft4222EepromNotProgrammed,
            ft4222InvalidArgs,
            ft4222NotSupported,
            ft4222OtherError,
            ft4222DeviceListNotReady,

            ft4222DeviceNotSupported = -100,        // FT_STATUS extending message (note that in LibFT4222.h these start from +1000 instead of -100)
            ft4222ClkNotSupported = -101,
            ft4222VenderCmdNotSupported = -102,
            ft4222IsNotSpiMode = -103,
            ft4222IsNotI2CMode = -104,
            ft4222IsNotSpiSingleMode = -105,
            ft4222IsNotSpiMultiMode = -106,
            ft4222WrongI2CAddr = -107,
            ft4222InvaildFunction = -108,
            ft4222InvalidPointer = -109,
            ft4222ExceededMaxTransferSize = -110,
            ft4222FailedToReadDevice = -111,
            ft4222I2CNotSupportedInThisMode = -112,
            ft4222GpioNotSupportedInThisMode = -113,
            ft4222GpioExceededMaxPortnum = -114,
            ft4222GpioWriteNotSupported = -115,
            ft4222GpioPullupInvalidInInputmode = -116,
            ft4222GpioPulldownInvalidInInputmode = -117,
            ft4222GpioOpendrainInvalidInOutputmode = -118,
            ft4222InterruptNotSupported = -119,
            ft4222GpioInputNotSupported = -120,
            ft4222EventNotSupported = -121,
            ft4222FunNotSupport = -122,
        };
/*
        static QString toString(FT_STATUS status)
        {
            switch (status)
            {
            case FT_OK: return "";
            case FT_INVALID_HANDLE: return "FT_INVALID_HANDLE";
            case FT_DEVICE_NOT_FOUND: return "FT_DEVICE_NOT_FOUND";
            case FT_DEVICE_NOT_OPENED: return "FT_DEVICE_NOT_OPENED";
            case FT_IO_ERROR: return "FT_IO_ERROR";
            case FT_INSUFFICIENT_RESOURCES: return "FT_INSUFFICIENT_RESOURCES";
            case FT_INVALID_PARAMETER: return "FT_INVALID_PARAMETER";
            case FT_INVALID_BAUD_RATE: return "FT_INVALID_BAUD_RATE";
            case FT_DEVICE_NOT_OPENED_FOR_ERASE: return "FT_DEVICE_NOT_OPENED_FOR_ERASE";
            case FT_DEVICE_NOT_OPENED_FOR_WRITE: return "FT_DEVICE_NOT_OPENED_FOR_WRITE";
            case FT_FAILED_TO_WRITE_DEVICE: return "FT_FAILED_TO_WRITE_DEVICE";
            case FT_EEPROM_READ_FAILED: return "FT_EEPROM_READ_FAILED";
            case FT_EEPROM_WRITE_FAILED: return "FT_EEPROM_WRITE_FAILED";
            case FT_EEPROM_ERASE_FAILED: return "FT_EEPROM_ERASE_FAILED";
            case FT_EEPROM_NOT_PRESENT: return "FT_EEPROM_NOT_PRESENT";
            case FT_EEPROM_NOT_PROGRAMMED: return "FT_EEPROM_NOT_PROGRAMMED";
            case FT_INVALID_ARGS: return "FT_INVALID_ARGS";
            case FT_NOT_SUPPORTED: return "FT_NOT_SUPPORTED";
            case FT_OTHER_ERROR: return "FT_OTHER_ERROR";
            case FT_DEVICE_LIST_NOT_READY: return "FT_DEVICE_LIST_NOT_READY";
            default:
                return "Unknown FT_* error";
            }
        }
*/
/*
        static Status from(FT4222_STATUS status)
        {
            // switch() is used so breakpoints could be placed despite that code quality and performance would be better using map<FT4222_STATUS, Status> instead
            switch (status)
            {
            case FT4222_OK:
                return Status::ok;
            case FT4222_INVALID_HANDLE:
                return Status::ft4222InvalidHandle;
            case FT4222_DEVICE_NOT_FOUND:
                return Status::ft4222DeviceNotFound;
            case FT4222_DEVICE_NOT_OPENED:
                return Status::ft4222DeviceNotOpened;
            case FT4222_IO_ERROR:
                return Status::ft4222IoError;
            case FT4222_INSUFFICIENT_RESOURCES:
                return Status::ft4222InsufficientResources;
            case FT4222_INVALID_PARAMETER:
                return Status::ft4222InvalidParameter;
            case FT4222_INVALID_BAUD_RATE:
                return Status::ft4222InvalidBaudRate;
            case FT4222_DEVICE_NOT_OPENED_FOR_ERASE:
                return Status::ft4222DeviceNotOpenedForErase;
            case FT4222_DEVICE_NOT_OPENED_FOR_WRITE:
                return Status::ft4222DeviceNotOpenedForWrite;
            case FT4222_FAILED_TO_WRITE_DEVICE:
                return Status::ft4222FailedToWriteDevice;
            case FT4222_EEPROM_READ_FAILED:
                return Status::ft4222EepromReadFailed;
            case FT4222_EEPROM_WRITE_FAILED:
                return Status::ft4222EepromWriteFailed;
            case FT4222_EEPROM_ERASE_FAILED:
                return Status::ft4222EepromEraseFailed;
            case FT4222_EEPROM_NOT_PRESENT:
                return Status::ft4222EepromNotPresent;
            case FT4222_EEPROM_NOT_PROGRAMMED:
                return Status::ft4222EepromNotProgrammed;
            case FT4222_INVALID_ARGS:
                return Status::ft4222InvalidArgs;
            case FT4222_NOT_SUPPORTED:
                return Status::ft4222NotSupported;
            case FT4222_OTHER_ERROR:
                return Status::ft4222OtherError;
            case FT4222_DEVICE_LIST_NOT_READY:
                return Status::ft4222DeviceListNotReady;
            case FT4222_DEVICE_NOT_SUPPORTED:
                return Status::ft4222DeviceNotSupported;
            case FT4222_CLK_NOT_SUPPORTED:
                return Status::ft4222ClkNotSupported;
            case FT4222_VENDER_CMD_NOT_SUPPORTED:
                return Status::ft4222VenderCmdNotSupported;
            case FT4222_IS_NOT_SPI_MODE:
                return Status::ft4222IsNotSpiMode;
            case FT4222_IS_NOT_I2C_MODE:
                return Status::ft4222IsNotI2CMode;
            case FT4222_IS_NOT_SPI_SINGLE_MODE:
                return Status::ft4222IsNotSpiSingleMode;
            case FT4222_IS_NOT_SPI_MULTI_MODE:
                return Status::ft4222IsNotSpiMultiMode;
            case FT4222_WRONG_I2C_ADDR:
                return Status::ft4222WrongI2CAddr;
            case FT4222_INVAILD_FUNCTION:
                return Status::ft4222InvaildFunction;
            case FT4222_INVALID_POINTER:
                return Status::ft4222InvalidPointer;
            case FT4222_EXCEEDED_MAX_TRANSFER_SIZE:
                return Status::ft4222ExceededMaxTransferSize;
            case FT4222_FAILED_TO_READ_DEVICE:
                return Status::ft4222FailedToReadDevice;
            case FT4222_I2C_NOT_SUPPORTED_IN_THIS_MODE:
                return Status::ft4222I2CNotSupportedInThisMode;
            case FT4222_GPIO_NOT_SUPPORTED_IN_THIS_MODE:
                return Status::ft4222GpioNotSupportedInThisMode;
            case FT4222_GPIO_EXCEEDED_MAX_PORTNUM:
                return Status::ft4222GpioExceededMaxPortnum;
            case FT4222_GPIO_WRITE_NOT_SUPPORTED:
                return Status::ft4222GpioWriteNotSupported;
            case FT4222_GPIO_PULLUP_INVALID_IN_INPUTMODE:
                return Status::ft4222GpioPullupInvalidInInputmode;
            case FT4222_GPIO_PULLDOWN_INVALID_IN_INPUTMODE:
                return Status::ft4222GpioPulldownInvalidInInputmode;
            case FT4222_GPIO_OPENDRAIN_INVALID_IN_OUTPUTMODE:
                return Status::ft4222GpioOpendrainInvalidInOutputmode;
            case FT4222_INTERRUPT_NOT_SUPPORTED:
                return Status::ft4222InterruptNotSupported;
            case FT4222_GPIO_INPUT_NOT_SUPPORTED:
                return Status::ft4222GpioNotSupportedInThisMode;
            case FT4222_EVENT_NOT_SUPPORTED:
                return Status::ft4222EventNotSupported;
            case FT4222_FUN_NOT_SUPPORT:
                return Status::ft4222FunNotSupport;
            default:
                return Status::ft4222UnknownStatus;
            }
        }
*/
/*
        static string toString(FT4222_STATUS s)
        {
            return toString(from(s));
        }


        static QString toString(Status status)
        {
            // switch() is used so breakpoints could be placed despite that code quality and performance would be better using map<Status, QString> instead
            switch (status)
            {
            case Status::ok:
                return "OK";
            case Status::unknownError:
                return "Unknown error";
            case Status::programmingError:
                return "Programming error";
            case Status::invalidDeviceId:
                return "Invalid device ID";
            case Status::timeout:
                return "I2C Timeout";
            case Status::fifoTimeout:
                return "Timeout while reading FIFO data";
            case Status::fifoFailure:
                return "Failure reading FIFO data";
            case Status::tooManyFrames:
                return "Too many FIFO frames reported";
            case Status::writeToDeviceRegisterNotAcknowledged:
                return "Write to device register not acknowledged";
            case Status::i2CmDataNack:
                return "I2C Master data not acknowledged during last operation";
            case Status::i2CmAddressNack:
                return "I2C Master slave address was not acknowledged during last operation";
            case Status::i2CmArbLost:
                return "I2C Master arbitration lost during last operation";
            case Status::ft4222UnknownStatus:
                return "Uknown FT4222 status";
            case Status::adcAllZeros:
                return "Invalid data detected on ADC (all zeros)";
            case Status::readFailure:
                return "I2C read failure";
            case Status::writeFailure:
                return "I2C write failure";
            case Status::accScaleError:
                return "ACC Scale Error";
            case Status::adcScaleError:
                return "ADC Scale Error";
            case Status::testConnectionFailed:
                return "Test Connection Failed";

            case Status::ft4222InvalidHandle:
                return "FT4222_INVALID_HANDLE";
            case Status::ft4222DeviceNotFound:
                return "FT4222_DEVICE_NOT_FOUND";
            case Status::ft4222DeviceNotOpened:
                return "FT4222_DEVICE_NOT_OPENED";
            case Status::ft4222IoError:
                return "FT4222_IO_ERROR";
            case Status::ft4222InsufficientResources:
                return "FT4222_INSUFFICIENT_RESOURCES";
            case Status::ft4222InvalidParameter:
                return "FT4222_INVALID_PARAMETER";
            case Status::ft4222InvalidBaudRate:
                return "FT4222_INVALID_BAUD_RATE";
            case Status::ft4222DeviceNotOpenedForErase:
                return "FT4222_DEVICE_NOT_OPENED_FOR_ERASE";
            case Status::ft4222DeviceNotOpenedForWrite:
                return "FT4222_DEVICE_NOT_OPENED_FOR_WRITE";
            case Status::ft4222FailedToWriteDevice:
                return "FT4222_FAILED_TO_WRITE_DEVICE";
            case Status::ft4222EepromReadFailed:
                return "FT4222_EEPROM_READ_FAILED";
            case Status::ft4222EepromWriteFailed:
                return "FT4222_EEPROM_WRITE_FAILED";
            case Status::ft4222EepromEraseFailed:
                return "FT4222_EEPROM_ERASE_FAILED";
            case Status::ft4222EepromNotPresent:
                return "FT4222_EEPROM_NOT_PRESENT";
            case Status::ft4222EepromNotProgrammed:
                return "FT4222_EEPROM_NOT_PROGRAMMED";
            case Status::ft4222InvalidArgs:
                return "FT4222_INVALID_ARGS";
            case Status::ft4222NotSupported:
                return "FT4222_NOT_SUPPORTED";
            case Status::ft4222OtherError:
                return "FT4222_OTHER_ERROR";
            case Status::ft4222DeviceListNotReady:
                return "FT4222_DEVICE_LIST_NOT_READY";
            case Status::ft4222DeviceNotSupported:
                return "FT4222_DEVICE_NOT_SUPPORTED";
            case Status::ft4222ClkNotSupported:
                return "FT4222_CLK_NOT_SUPPORTED";
            case Status::ft4222VenderCmdNotSupported:
                return "FT4222_VENDER_CMD_NOT_SUPPORTED";
            case Status::ft4222IsNotSpiMode:
                return "FT4222_IS_NOT_SPI_MODE";
            case Status::ft4222IsNotI2CMode:
                return "FT4222_IS_NOT_I2C_MODE";
            case Status::ft4222IsNotSpiSingleMode:
                return "FT4222_IS_NOT_SPI_SINGLE_MODE";
            case Status::ft4222IsNotSpiMultiMode:
                return "FT4222_IS_NOT_SPI_MULTI_MODE";
            case Status::ft4222WrongI2CAddr:
                return "FT4222_WRONG_I2C_ADDR";
            case Status::ft4222InvaildFunction:
                return "FT4222_INVAILD_FUNCTION";
            case Status::ft4222InvalidPointer:
                return "FT4222_INVALID_POINTER";
            case Status::ft4222ExceededMaxTransferSize:
                return "FT4222_EXCEEDED_MAX_TRANSFER_SIZE";
            case Status::ft4222FailedToReadDevice:
                return "FT4222_FAILED_TO_READ_DEVICE";
            case Status::ft4222I2CNotSupportedInThisMode:
                return "FT4222_I2C_NOT_SUPPORTED_IN_THIS_MODE";
            case Status::ft4222GpioNotSupportedInThisMode:
                return "FT4222_GPIO_NOT_SUPPORTED_IN_THIS_MODE";
            case Status::ft4222GpioExceededMaxPortnum:
                return "FT4222_GPIO_EXCEEDED_MAX_PORTNUM";
            case Status::ft4222GpioWriteNotSupported:
                return "FT4222_GPIO_WRITE_NOT_SUPPORTED";
            case Status::ft4222GpioPullupInvalidInInputmode:
                return "FT4222_GPIO_PULLUP_INVALID_IN_INPUTMODE";
            case Status::ft4222GpioPulldownInvalidInInputmode:
                return "FT4222_GPIO_PULLDOWN_INVALID_IN_INPUTMODE";
            case Status::ft4222GpioOpendrainInvalidInOutputmode:
                return "FT4222_GPIO_OPENDRAIN_INVALID_IN_OUTPUTMODE";
            case Status::ft4222InterruptNotSupported:
                return "FT4222_INTERRUPT_NOT_SUPPORTED";
            case Status::ft4222GpioInputNotSupported:
                return "FT4222_GPIO_INPUT_NOT_SUPPORTED";
            case Status::ft4222EventNotSupported:
                return "FT4222_EVENT_NOT_SUPPORTED";
            case Status::ft4222FunNotSupport:
                return "FT4222_FUN_NOT_SUPPORT";
            default:
                return "Unknown status: " + QString::number(int(status));
            }
        }
        */

    private:

    };


#endif // FT_HPP
