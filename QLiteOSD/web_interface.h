/*
 *  QLiteOSD is an simple OSD for DJI FPV System:
 *  This is an Arduino project that handles basic OSD functions
 *  from BMP280 to a Simple Voltage Sensor to feed it 
 *  to DJI FPV System.
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2023 David Payne
 * 
 * This software is based on and uses software published by Paul Kurucz (pkuruz):opentelem_to_bst_bridge
 * as well as software d3ngit : djihdfpv_mavlink_to_msp_V2 and crashsalot : VOT_to_DJIFPV
 * 
 * License info: See the LICENSE file at the repo top level
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */


static const char HEAD_TITLE[] PROGMEM = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'>"
                                        "<meta http-equiv='Content-Type' content='text/html; charset=UTF-8' />"
                                        "<link rel='icon' href='data:;base64,='>"
                                        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                                        "<title>QLiteOSD</title>"
                                        "	<style>"
                                        "        body {"
                                        "            font-family: Arial, sans-serif;"
                                        "            margin: 0;"
                                        "            padding: 0;"
                                        "        }"
                                        "        header {"
                                        "            background-color: #4CAF50;"
                                        "            color: white;"
                                        "            padding: 1em 0;"
                                        "            text-align: center;"
                                        "        }"
                                        "        nav {"
                                        "            background-color: #333;"
                                        "            display: flex;"
                                        "            justify-content: space-between;"
                                        "            align-items: left, top;"
                                        "            padding: 0 16px;"
                                        "        }"
                                        "        nav ul {"
                                        "            list-style-type: none;"
                                        "            margin: 0;"
                                        "            padding: 0;"
                                        "            overflow: hidden;"
                                        "            display: flex;"
                                        "            flex-direction: row;"
                                        "        }"
                                        "        nav ul li {"
                                        "            display: inline;"
                                        "        }"
                                        "        nav ul li a {"
                                        "            display: block;"
                                        "            color: white;"
                                        "            text-align: left;"
                                        "            padding: 14px 16px;"
                                        "            text-decoration: none;"
                                        "        }"
                                        "        nav ul li a:hover {"
                                        "            background-color: #111;"
                                        "        }"
                                        "        .menu-icon {"
                                        "            display: none;"
                                        "            padding: 14px 16px;"
                                        "            cursor: pointer;"
                                        "        }"
                                        "        .menu-icon span {"
                                        "            display: block;"
                                        "            width: 25px;"
                                        "            height: 3px;"
                                        "            margin: 5px auto;"
                                        "            background-color: white;"
                                        "        }"
                                        "        @media (max-width: 600px) {"
                                        "            nav ul {"
                                        "                display: none;"
                                        "                flex-direction: column;"
                                        "                width: 100%;"
                                        "            }"
                                        "            nav ul li {"
                                        "                float: none;"
                                        "                width: 100%;"
                                        "            }"
                                        "            .menu-icon {"
                                        "                display: block;"
                                        "            }"
                                        "        }"
                                        "        .show {"
                                        "            display: flex !important;"
                                        "        }"
                                        "        section {"
                                        "            padding: 20px;"
                                        "        }"
                                        "        form button {"
                                        "            width: 100%;"
                                        "            background-color: #4CAF50;"
                                        "            color: white;"
                                        "            padding: 14px 20px;"
                                        "            margin: 10px 0;"
                                        "            border: none;"
                                        "            border-radius: 4px;"
                                        "            cursor: pointer;"
                                        "        }"
                                        "        form button:hover {"
                                        "            background-color: #45a049;"
                                        "        }"
                                        "    </style>"
                                        " </head>";

static const char BODY_MENU[] PROGMEM = "<body>"
                                        "    <header>"
                                        "        <h1>QLiteOSD</h1>"
                                        "    </header>"
                                        "    <nav>"
                                        "        <div class='menu-icon' onclick='toggleMenu()'>"
                                        "            <span></span>"
                                        "            <span></span>"
                                        "            <span></span>"
                                        "        </div>"
                                        "        <ul id='menu'>"
                                        "            <li><a href='/'>Home</a></li>"
                                        "            <li><a href='/configure'>Configure</a></li>"
                                        "            <li><a href='/wifioff' onclick='return confirm(\"Do you want to turn off Wifi Access Point and start OSD again?\")'>Turn Off Wifi</a></li>"
                                        "            <li><a href='/reset' onclick='return confirm(\"Do you want to delete all logs and reset to default settings?\")'>Reset</a></li>"
                                        "        </ul>"
                                        "    </nav>"
                                        "<section><p>";
                                      
static const char BODY_END[] PROGMEM = "<br/><br/><br/><hr/>QLiteOSD v%VERSION%</p></section>"
                                          "    <script>"
                                          "        function toggleMenu() {"
                                          "            var menu = document.getElementById('menu');"
                                          "            if (menu.classList.contains('show')) {"
                                          "                menu.classList.remove('show');"
                                          "            } else {"
                                          "                menu.classList.add('show');"
                                          "            }"
                                          "        }"
                                          "    </script>"
                                          "</body></html>";

static const char CONFIG_FORM[] PROGMEM = "<h2>Configuration:</h2><form action='/saveconfig' method='get'>"
                          "<p><label>Craft Name</label><input type='text' name='craftname_form' value='%CRAFTNAME%' maxlength='14'></p>"
                          "<p><input name='use_imperial_form' type='checkbox' %USEIMPERIALCHECKED%> Use Imperial Units</p>"
                          "<p><input name='use_pwm_arm_form' type='checkbox' %USEPWMCHECKED%> Arm with PWM Switch (D5 pin)</p>"
                          "<button type='submit'>Save</button></form>";                                          
