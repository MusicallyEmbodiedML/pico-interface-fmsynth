#include "hardware/adc.h"
#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"
#include <sstream>
#include <iomanip> // For std::fixed and std::setprecision

#include "common_defs.h"
#include "MEMLSerial_Pico.hpp"


#if 0

std::string stateToJson(const ts_app_state& state) {
    std::ostringstream oss;
    oss << "{\n"
        << "  \"n_iterations\": " << state.n_iterations << ",\n"
        << "  \"last_error\": " << std::fixed << std::setprecision(6) << state.last_error << ",\n"
        << "  \"exploration_range\": " << state.exploration_range << ",\n"
        << "  \"app_id\": " << static_cast<int>(state.app_id) << ",\n"
        << "  \"current_dataset\": " << static_cast<int>(state.current_dataset) << ",\n"
        << "  \"current_model\": " << static_cast<int>(state.current_model) << ",\n"
        << "  \"current_nn_mode\": " << static_cast<int>(state.current_nn_mode) << ",\n"
        << "  \"current_expl_mode\": " << static_cast<int>(state.current_expl_mode) << "\n"
        << "}";
    return oss.str();
}


// max length of the tags defaults to be 8 chars
// LWIP_HTTPD_MAX_TAG_NAME_LEN
const char * __not_in_flash("httpd") ssi_example_tags[] = {
    "Hello",    // 0
    "counter",  // 1
    "GPIO",     // 2
    "state1",   // 3
    "state2",   // 4
    "state3",   // 5
    "state4",   // 6
    "bg1",      // 7
    "bg2",      // 8
    "bg3",      // 9
    "bg4",       // 10
    "JSONDATA", //11
};

u16_t __time_critical_func(ssi_handler)(int iIndex, char *pcInsert, int iInsertLen)
{
    size_t printed;
    switch (iIndex) {
        case 0: /* "Hello" */
            printed = snprintf(pcInsert, iInsertLen, "Hello user number %d!", rand());
            break;
        case 1: /* "counter" */
        {
            static int counter;
            counter++;
            printed = snprintf(pcInsert, iInsertLen, "%d", counter);
        }
            break;
        case 2: /* "GPIO" */
        {
            const float voltage = adc_read() * 3.3f / (1 << 12);
            printed = snprintf(pcInsert, iInsertLen, "%f", voltage);
        }
            break;
        case 3: /* "state1" */
        case 4: /* "state2" */
        case 5: /* "state3" */
        case 6: /* "state4" */
        {
            bool state;
            if(iIndex == 3)
                state = gpio_get(LED1);
            else if(iIndex == 4)
                state = gpio_get(LED2);
            else if(iIndex == 5)
                state = gpio_get(LED3);
            else if(iIndex == 6)
                state = gpio_get(LED4);

            if(state)
                printed = snprintf(pcInsert, iInsertLen, "checked");
            else
                printed = snprintf(pcInsert, iInsertLen, " ");
        }
          break;

        case 7:  /* "bg1" */
        case 8:  /* "bg2" */
        case 9:  /* "bg3" */
        case 10: /* "bg4" */
        {
            bool state;
            if(iIndex == 7)
                state = gpio_get(LED1);
            else if(iIndex == 8)
                state = gpio_get(LED2);
            else if(iIndex == 9)
                state = gpio_get(LED3);
            else if(iIndex == 10)
                state = gpio_get(LED4);

            if(state)
                printed = snprintf(pcInsert, iInsertLen, "\"background-color:green;\"");
            else
                printed = snprintf(pcInsert, iInsertLen, "\"background-color:red;\"");
        }
          break;
        case 11:
        {
            std::string json_state = stateToJson(GAppState);
            std::cout << "iInsertLen: " << iInsertLen << std::endl;
            std::cout << "JSON length: " << json_state.size() << std::endl;
            std::cout << "LWIP_HTTPD_MAX_TAG_NAME_LEN" << LWIP_HTTPD_MAX_TAG_NAME_LEN << std::endl;
            printed=snprintf(pcInsert, iInsertLen, json_state.c_str());
        }
        break;
        default: /* unknown tag */
            printed = 0;
            break;
    }
      LWIP_ASSERT("sane length", printed <= 0xFFFF);
      return (u16_t)printed;
}


void ssi_init()
{
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    for (size_t i = 0; i < LWIP_ARRAYSIZE(ssi_example_tags); i++) {
        LWIP_ASSERT("tag too long for LWIP_HTTPD_MAX_TAG_NAME_LEN",
                    strlen(ssi_example_tags[i]) <= LWIP_HTTPD_MAX_TAG_NAME_LEN);
    }

      http_set_ssi_handler(ssi_handler,
                           ssi_example_tags, LWIP_ARRAYSIZE(ssi_example_tags)
      );
}

#endif

const char * __not_in_flash("httpd") ssi_example_tags[] = {
    "NITER", // 0
    "LERR",  // 1
    "EXRG",  // 2
    "APID",  // 3
    "CDAT",  // 4
    "CMOD",  // 5
    "NNMD",  // 6
    "EXMD"   // 7
};

u16_t __time_critical_func(ssi_handler)(int iIndex, char *pcInsert, int iInsertLen) {
    if (pcInsert == nullptr || iInsertLen <= 0) {
        return 0; // Invalid parameters, nothing written
    }

    switch (iIndex) {
        case 0: // "NITER"
            return snprintf(pcInsert, iInsertLen, "%u", GAppState.n_iterations);
        case 1: // "LERR"
            return snprintf(pcInsert, iInsertLen, "%.6f", GAppState.last_error);
        case 2: // "EXRG"
            return snprintf(pcInsert, iInsertLen, "%.6f", GAppState.exploration_range);
        case 3: // "APID"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(GAppState.app_id));
        case 4: // "CDAT"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(GAppState.current_dataset));
        case 5: // "CMOD"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(GAppState.current_model));
        case 6: // "NNMD"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(GAppState.current_nn_mode));
        case 7: // "EXMD"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(GAppState.current_expl_mode));
        default:
            return 0; // Unknown index, nothing written
    }
}

void ssi_init()
{
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    for (size_t i = 0; i < LWIP_ARRAYSIZE(ssi_example_tags); i++) {
        LWIP_ASSERT("tag too long for LWIP_HTTPD_MAX_TAG_NAME_LEN",
                    strlen(ssi_example_tags[i]) <= LWIP_HTTPD_MAX_TAG_NAME_LEN);
    }

      http_set_ssi_handler(ssi_handler,
                           ssi_example_tags, LWIP_ARRAYSIZE(ssi_example_tags)
      );
}
