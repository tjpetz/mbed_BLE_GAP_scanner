/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mbed.h>
#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"

using namespace std::chrono;
using std::milli;
using namespace std::literals::chrono_literals;


/* Scanning happens repeatedly and is defined by:
 *  - The scan interval which is the time (in 0.625us) between each scan cycle.
 *  - The scan window which is the scanning time (in 0.625us) during a cycle.
 * If the scanning process is active, the local device sends scan requests
 * to discovered peer to get additional data.
 */
static const ble::ScanParameters scan_params(
    ble::phy_t::LE_CODED,
    ble::scan_interval_t(80),
    ble::scan_window_t(60),
    false /* active scanning */
);

/* config end */

events::EventQueue event_queue;

REDIRECT_STDOUT_TO(Serial);

// mbed::USBSerial myUSBSerial(false);

// // Redirect the console to the USBSerial port
// mbed::FileHandle *mbed::mbed_override_console(int fd) {
//   return SerialUSB;
// }

inline void print_error(ble_error_t err, const char* msg) {
  printf("%s: %s", msg, BLE::errorToString(err));
}

/** print device address to the terminal */
inline void print_address(const ble::address_t &addr)
{
    printf("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

inline void print_mac_address()
{
    /* Print out device MAC address to the console*/
    ble::own_address_type_t addr_type;
    ble::address_t address;
    BLE::Instance().gap().getAddress(addr_type, address);
    printf("DEVICE MAC ADDRESS: ");
    print_address(address);
}

inline const char* phy_to_string(ble::phy_t phy) {
    switch(phy.value()) {
        case ble::phy_t::LE_1M:
            return "LE 1M";
        case ble::phy_t::LE_2M:
            return "LE 2M";
        case ble::phy_t::LE_CODED:
            return "LE coded";
        default:
            return "invalid PHY";
    }
}


/** Demonstrate advertising, scanning and connecting */
class GapDemo : private mbed::NonCopyable<GapDemo>, public ble::Gap::EventHandler
{
public:
    GapDemo(BLE& ble, events::EventQueue& event_queue) :
        _ble(ble),
        _gap(ble.gap()),
        _event_queue(event_queue)
    {
    }

    ~GapDemo()
    {
        if (_ble.hasInitialized()) {
            _ble.shutdown();
        }
    }

    /** Start BLE interface initialisation */
    void run()
    {
        /* handle gap events */
        _gap.setEventHandler(this);

        ble_error_t error = _ble.init(this, &GapDemo::on_init_complete);
        if (error) {
            print_error(error, "Error returned by BLE::init");
            return;
        }

        /* this will not return until shutdown */
        _event_queue.dispatch_forever();
    }

private:
    /** This is called when BLE interface is initialised and starts the first mode */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *event)
    {
        if (event->error) {
            print_error(event->error, "Error during the initialisation");
            return;
        }

        print_mac_address();

        /* setup the default phy used in connection to 2M to reduce power consumption */
        if (_gap.isFeatureSupported(ble::controller_supported_features_t::LE_2M_PHY)) {
            ble::phy_set_t phys(/* 1M */ false, /* 2M */ true, /* coded */ false);

            ble_error_t error = _gap.setPreferredPhys(/* tx */&phys, /* rx */&phys);

            /* PHY 2M communication will only take place if both peers support it */
            if (error) {
                print_error(error, "GAP::setPreferedPhys failed");
            } else {
              printf("Upgrade PHY to 2M\n");
            }


        } else {
            /* otherwise it will use 1M by default */
        }

        /* all calls are serialised on the user thread through the event queue */
        _event_queue.call(this, &GapDemo::scan);
    }

    /** Set up and start scanning */
    void scan()
    {

        // Build the scan parameters

        ble_error_t error = _gap.setScanParameters(
          ble::ScanParameters()
            .setPhys(true, true)
            .setCodedPhyConfiguration(ble::scan_interval_t(80), ble::scan_window_t(60), false)
            .set1mPhyConfiguration(ble::scan_interval_t(100), ble::scan_window_t(40), false)
            );
        if (error) {
            print_error(error, "Error caused by Gap::setScanParameters");
            return;
        }

        /* start scanning and attach a callback that will handle advertisements
         * and scan requests responses */
        error = _gap.startScan();
        if (error) {
            print_error(error, "Error caused by Gap::startScan");
            return;
        }

        printf(
               "\r\nScanning started (interval: %dms, window: %dms).\r\n",
               scan_params.get1mPhyConfiguration().getInterval().valueInMs(),
               scan_params.get1mPhyConfiguration().getWindow().valueInMs());
        

    }

private:
    /* Gap::EventHandler */

    /** Report on found devices */
    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) override
    {
        printf("Scan found: %02x:%02x:%02x:%02x:%02x:%02x, Phy = %s, Tx Power = %d, RSSI = %d\n",
          event.getPeerAddress()[5], event.getPeerAddress()[4], event.getPeerAddress()[3],
          event.getPeerAddress()[2], event.getPeerAddress()[1], event.getPeerAddress()[0], 
          phy_to_string(event.getPrimaryPhy()), event.getTxPower(), event.getRssi());
        
        ble::AdvertisingDataParser adv_parser(event.getPayload());

        /* parse the advertising payload*/
        while (adv_parser.hasNext()) {
          ble::AdvertisingDataParser::element_t field = adv_parser.next();

          if (field.type == ble::adv_data_type_t::FLAGS) {
            printf("\tFlags = ");
            ble::adv_data_flags_t flag(field.value.data()[0]);
            if (flag.getGeneralDiscoverable())
              printf("LE_GENERAL_DISCOVERABLE ");
            if (flag.getlimitedDiscoverable())
              printf("LE_LIMITED_DISCOVERABLE ");
            if (flag.getBrEdrNotSupported())
              printf("BREDR_NOT_SUPPORTED ");
            if (flag.getSimultaneousLeBredrC())
              printf("SIMULTANEOUS_LE_BREDR_C ");
            if (flag.getSimultaneousLeBredrH())
              printf("SIMULTANEOUS_LE_BREDR_H ");
            printf("\n");                
          } else if (field.type == ble::adv_data_type_t::INCOMPLETE_LIST_16BIT_SERVICE_IDS) {
            printf("\tIncomplete List of 16bit Service IDs = ");
            for (int i = 0; i < field.value.size(); i +=2) {
              printf("%02x%02x", field.value.data()[1], field.value.data()[0]);
            }
            printf("\n");
          } else if (field.type == ble::adv_data_type_t::COMPLETE_LIST_16BIT_SERVICE_IDS) {
            printf("\tComplete List of 16bit Service IDs = ");
            for (int i = 0; i < field.value.size(); i +=2) {
              printf("%02x%02x", field.value.data()[1], field.value.data()[0]);
            }
            printf("\n");
          } else if (field.type == ble::adv_data_type_t::INCOMPLETE_LIST_32BIT_SERVICE_IDS) {
            printf("\tIncomplete List of 32bit Service IDs\n");
          } else if (field.type == ble::adv_data_type_t::COMPLETE_LIST_32BIT_SERVICE_IDS) {
            printf("\tComplete List of 32bit Service IDs\n");
          } else if (field.type == ble::adv_data_type_t::INCOMPLETE_LIST_128BIT_SERVICE_IDS) {
            printf("\tIncomplete List of 128bit Service IDs\n");
          } else if (field.type == ble::adv_data_type_t::COMPLETE_LIST_128BIT_SERVICE_IDS) {
            printf("\tComplete List of 128bit Service IDs\n");
          } else if (field.type == ble::adv_data_type_t::SHORTENED_LOCAL_NAME) {
            char localname[128];
            memset(localname, 0, 128);
            memcpy(localname, field.value.data(), field.value.size());
            printf("\tShort Name = %s\n", localname);
          } else if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME) {
            char localname[128];
            memset(localname, 0, 128);
            memcpy(localname, field.value.data(), field.value.size());
            printf("\tComplete Name = %s\n", localname);
          } else if (field.type == ble::adv_data_type_t::TX_POWER_LEVEL) {
              printf("\tAdvertised TX Power Level = %d\n", field.value.data()[0]);
          } else if (field.type == ble::adv_data_type_t::DEVICE_ID) {
            printf("\tDevice ID = %d", field.value.size());
          } else if (field.type == ble::adv_data_type_t::SLAVE_CONNECTION_INTERVAL_RANGE) {
            printf("\tHas SLAVE_CONNECTION_INTERVAL_RANGE\n");
          } else if (field.type == ble::adv_data_type_t::LIST_16BIT_SOLICITATION_IDS) {
            printf("\tHas SLAVE_CONNECTION_INTERVAL_RANGE\n");
          } else if (field.type == ble::adv_data_type_t::LIST_128BIT_SOLICITATION_IDS) {
            printf("\tHas LIST_128BIT_SOLICITATION_IDS\n");
          } else if (field.type == ble::adv_data_type_t::SERVICE_DATA) {
            printf("\tHas service data\n");
          } else if (field.type == ble::adv_data_type_t::SERVICE_DATA_16BIT_ID) {
            printf("\tHas SERVICE_DATA_16BIT_ID\n");
          } else if (field.type == ble::adv_data_type_t::SERVICE_DATA_128BIT_ID) {
            printf("\tHas SERVICE_DATA_128BIT_ID\n");
          } else if (field.type == ble::adv_data_type_t::APPEARANCE) {
            printf("\tAppearance = %d\n", field.value.data()[0]);
          } else if (field.type == ble::adv_data_type_t::ADVERTISING_INTERVAL) {
            printf("\tHas ADVERTISING_INTERVAL\n");
          } else if (field.type == ble::adv_data_type_t::MANUFACTURER_SPECIFIC_DATA) {
            printf("\tHas MANUFACTURER_SPECIFIC_DATA\n");
          } 
        }
    }

    void onScanTimeout(const ble::ScanTimeoutEvent&) override
    {
        printf("Stopped scanning due to timeout parameter\r\n");
        _event_queue.call(this, &GapDemo::scan);
    }

    /** This is called by Gap to notify the application we connected,
     *  in our case it immediately disconnects */
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override
    {
    }

    /** This is called by Gap to notify the application we disconnected,
     *  in our case it calls next_demo_mode() to progress the demo */
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override
    {
        printf("Disconnected\r\n");
    }

    /**
     * Implementation of Gap::EventHandler::onReadPhy
     */
    void onReadPhy(
        ble_error_t error,
        ble::connection_handle_t connectionHandle,
        ble::phy_t txPhy,
        ble::phy_t rxPhy
    ) override
    {
        if (error) {
            printf(
                "Phy read on connection %d failed with error code %s\r\n",
                connectionHandle, BLE::errorToString(error)
            );
        } else {
            printf(
                "Phy read on connection %d - Tx Phy: %s, Rx Phy: %s\r\n",
                connectionHandle, phy_to_string(txPhy), phy_to_string(rxPhy)
            );
        }
    }

    /**
     * Implementation of Gap::EventHandler::onPhyUpdateComplete
     */
    void onPhyUpdateComplete(
        ble_error_t error,
        ble::connection_handle_t connectionHandle,
        ble::phy_t txPhy,
        ble::phy_t rxPhy
    ) override
    {
        if (error) {
            printf(
                "Phy update on connection: %d failed with error code %s\r\n",
                connectionHandle, BLE::errorToString(error)
            );
        } else {
            printf(
                "Phy update on connection %d - Tx Phy: %s, Rx Phy: %s\r\n",
                connectionHandle, phy_to_string(txPhy), phy_to_string(rxPhy)
            );
        }
    }

    /**
     * Implementation of Gap::EventHandler::onDataLengthChange
     */
    void onDataLengthChange(
        ble::connection_handle_t connectionHandle,
        uint16_t txSize,
        uint16_t rxSize
    ) override
    {
        printf(
            "Data length changed on the connection %d.\r\n"
            "Maximum sizes for over the air packets are:\r\n"
            "%d octets for transmit and %d octets for receive.\r\n",
            connectionHandle, txSize, rxSize
        );
    }

private:

private:
    BLE &_ble;
    ble::Gap &_gap;
    events::EventQueue &_event_queue;

#if BLE_FEATURE_EXTENDED_ADVERTISING
    ble::advertising_handle_t _extended_adv_handle = ble::INVALID_ADVERTISING_HANDLE;
#endif // BLE_FEATURE_EXTENDED_ADVERTISING
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(mbed::Callback<void()>(&context->ble, &BLE::processEvents));
}

void setup()
{
    delay(7000);

    BLE &ble = BLE::Instance();

    /* this will inform us off all events so we can schedule their handling
     * using our event queue */
    ble.onEventsToProcess(schedule_ble_events);

    GapDemo demo(ble, event_queue);

    demo.run();
}

void loop() {
  // put your main code here, to run repeatedly:

}
