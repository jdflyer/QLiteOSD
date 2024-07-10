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
                                        "            border-radius: 8px;"
                                        "            cursor: pointer;"
                                        "        }"
                                        "        form button:hover {"
                                        "            background-color: #45a049;"
                                        "        }"
                                        "        .group {"
                                        "            margin-bottom: 20px;"
                                        "        }"
                                        "        table { width: 100%; border-collapse: collapse; margin-top: 20px; }"
                                        "        .item { margin-left: 20px; display: flex; align-items: center; }"
                                        "        .readonly-field { margin-left: 10px; display: block; width: 75px; }"
                                        "        th, td { border: 1px solid #000; text-align: center; padding: 5px; cursor: pointer; }"
                                        "        th { background-color: #f2f2f2; }"
                                        "        .selected-cell { background-color: #FFD700; /* Gold background for selected cells */ }"
                                        "        .selected-item { background-color: #90EE90; /* Light Green background for selected items */ }"
                                        "		.slider-container {"
                                        "            display: flex;"
                                        "            justify-content: center;"
                                        "            align-items: center;"
                                        "			text-align: right;"
                                        "            margin: 10px 0;"
                                        "        }"
                                        "        .slider-container label {"
                                        "            margin-right: 10px;"
                                        "            width: 50px;"
                                        "        }"
                                        "        .slider {"
                                        "            width: 300px;"
                                        "        }"
                                        "        #color-display {"
                                        "            margin: 20px auto;"
                                        "            width: 300px;"
                                        "            height: 60px;"
                                        "            border: 1px solid #000;"
                                        "            background-color: rgb(0, 0, 0);"
                                        "        }"
                                        "		#rgb-section {"
                                        "            display: none;"
                                        "            margin-top: 20px;"
                                        "            padding: 20px;"
                                        "            border: 1px solid #000;"
                                        "            background-color: #f0f0f0;"
                                        "        }"
                                        "		.rgb-input {"
                                        "			width: 30px;"
                                        "		}"
                                        "    </style>";

static const char BODY_MENU[] PROGMEM = "</head><body>"
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
                                        "            <li><a href='/osd'>OSD Layout</a></li>"
                                        "            <li><a href='#' onclick='if (confirm(\"Do you want to turn off Wifi Access Point and start OSD again?\")){fetch(\"/wifioff\");}'>Turn Off Wifi</a></li>"
                                        "            <li><a href='#' onclick='if (confirm(\"Do you want to delete all logs and reset to default settings?\")){fetch(\"/reset\");}'>Reset</a></li>"
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
                          "<p><label for='voltage'>Adjust Voltage: </label>"
			                    "<input type='text' name='voltage' id='voltage' readonly></p>"
                          "<p><label for='numericInput'>Board VCC (3.00V to 6.00V): </label>"
                          "<input type='number' id='numericInput' value='%VCC%' name='numericInput' min='3.00' max='6.00' step='0.01' required></p>"
                          "<p><label for='toggle-select'>Select LED mode:</label>"
                          "<select id='toggle-select' name='rgbmode'>"
                          "%RGB_OPTIONS%"
                          "</select> (D6 pin)</p>"
                          "	<div id='rgb-section'>"
                          "		<div class='slider-container'>"
                          "			<label for='red'>Red</label>"
                          "			<input type='range' id='red' class='slider' min='0' max='255' value='%RED%'>"
                          "			<input type='text' name='red' id='red-input' class='rgb-input' readonly>"
                          "		</div>"
                          "		<div class='slider-container'>"
                          "			<label for='green'>Green</label>"
                          "			<input type='range' id='green' class='slider' min='0' max='255' value='%GREEN%'>"
                          "			<input type='text' name='green' id='green-input' class='rgb-input' readonly>"
                          "		</div>"
                          "		<div class='slider-container'>"
                          "			<label for='blue'>Blue</label>"
                          "			<input type='range' id='blue' class='slider' min='0' max='255' value='%BLUE%'>"
                          "			<input type='text' name='blue' id='blue-input' class='rgb-input' readonly>"
                          "		</div>"
                          "		<div id='color-display'></div>"
                          "	</div>"
                          "<button type='submit'>Save</button></form>";

static const char RGB_OPTIONS[] PROGMEM = "<option value='OFF'>OFF</option>"
                          "    <option value='ON'>ON</option>"
                          "    <option value='STROBE'>STROBE</option>"
                          "    <option value='ALTITUDE'>ALTITUDE</option>"
                          "    <option value='BATTERY'>BATTERY</option>";

static const char RGB_JS[] PROGMEM = "	<script>"
                                    "		const redSlider = document.getElementById('red');"
                                    "		const greenSlider = document.getElementById('green');"
                                    "		const blueSlider = document.getElementById('blue');"
                                    "		const colorDisplay = document.getElementById('color-display');"
                                    "		const redValue = document.getElementById('red-input');"
                                    "		const greenValue = document.getElementById('green-input');"
                                    "		const blueValue = document.getElementById('blue-input');"
                                    "		const toggleSelect = document.getElementById('toggle-select');"
                                    "   const displaySection = document.getElementById('rgb-section');"
                                    "		function updateColor() {"
                                    "			const red = redSlider.value;"
                                    "			const green = greenSlider.value;"
                                    "			const blue = blueSlider.value;"
                                    "			const rgbColor = \`rgb(${red}, ${green}, ${blue})\`;"
                                    "			colorDisplay.style.backgroundColor = rgbColor;"
                                    "			redValue.value = red;"
                                    "			greenValue.value = green;"
                                    "			blueValue.value = blue;"
                                    "		}"
                                    "		redSlider.addEventListener('input', updateColor);"
                                    "		greenSlider.addEventListener('input', updateColor);"
                                    "		blueSlider.addEventListener('input', updateColor);"
                                    "		updateColor();"
                                    "   toggleSelect.addEventListener('change', function() {"
                                    "     if (toggleSelect.value === 'ON' || toggleSelect.value === 'STROBE') {"
                                    "       displaySection.style.display = 'block';"
                                    "     } else {"
                                    "       displaySection.style.display = 'none';"
                                    "     }"
                                    "   });"
                                    "   if (toggleSelect.value === 'ON' || toggleSelect.value === 'STROBE') {"
                                    "      displaySection.style.display = 'block';"
                                    "   } else {"
                                    "     displaySection.style.display = 'none';"
                                    "   }"
                                    "	</script>";      

static const char VOLT_JS[] PROGMEM = "<script>"
                                      "		document.getElementById('numericInput').addEventListener('input', calculateVoltage);"
                                      "		function calculateVoltage() {"
                                      "			const R1 = %R1%;"
                                      "			const R2 = %R2%;"
                                      "			const readValue = %READVALUE%;"
                                      "			const arduinoVCC = parseFloat(document.getElementById('numericInput').value);"
                                      "			var vbat = (readValue * (arduinoVCC / 1024.0)) * (1 + (R2 / R1));"
                                      "			document.getElementById('voltage').value = vbat.toFixed(2);"
                                      "		}"
                                      "		calculateVoltage();"
                                      "</script>";

static const char OSD_JS[] PROGMEM =    "<script>function handleCellClick(e){let t=document.querySelector('input[name=\"item\"]:checked');"
                                        "if(t){let l=t.getAttribute('data-item-number'),r=t.id,a=e.target.getAttribute('data-value');if(11>=e.target.textContent.trim()){alert('This cell is already occupied. Please choose another cell.');"
                                        "return}let n=document.querySelectorAll('.readonly-field');for(let o of n)if(o.value===a&&234!=a){alert('This value is already assigned to another OSD item. Please choose another cell.');"
                                        "return}let i=document.querySelectorAll('td');for(let c of i)if(c.innerText===l){c.style.backgroundColor='#FFFFFF',c.innerText=c.getAttribute('data-value');"
                                        "break}234!=e.target.textContent&&(e.target.textContent=l,e.target.classList.add('selected-cell'),e.target.style.backgroundColor=getColorByNumber(l));"
                                        "let d=document.getElementById(r+'Field');d.value=a,d.style.backgroundColor='',234!=e.target.textContent&&(d.style.backgroundColor=getColorByNumber(l))}else alert('Please select an OSD item first.')}"
                                        "function getColorByNumber(e){let t=['#F08080','#F4A460','#FAFAD2','#EEE8AA','#90EE90','#7FFFD4','#E0FFFF','#87CEFA','#6495ED','#DDA0DD','#D8BFD8'];"
                                        "return t[(e-1)%t.length]}function handleRadioChange(e){let t=document.querySelectorAll('.item');t.forEach(e=>{e.classList.remove('selected-item')});"
                                        "let l=e.target.parentElement;l.classList.add('selected-item')}function init(){let e=document.querySelectorAll('td');"
                                        "e.forEach(e=>{e.addEventListener('click',handleCellClick)});let t=document.querySelectorAll('input[name=\"item\"]');t.forEach(e=>{e.addEventListener('change',handleRadioChange)})}window.onload=init;</script>";
                          
static const char OSD_DISPLAY[] PROGMEM = "    <h1>OSD Layout</h1>"
                                  "    <div class='group'>"
                                  "        <h2>Select an OSD Item</h2><form action='/saveosd' method='get'>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item1' name='item' value='altitude' data-item-number='1'>"
                                  "            <label for='item1'>1. Altitude</label>"
                                  "            <input name='altitude_pos' value='%ALT_VAL%' type='text' id='item1Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item2' name='item' value='cellVoltage' data-item-number='2'>"
                                  "            <label for='item2'>2. Cell Voltage</label>"
                                  "            <input name='cell_voltage_pos' value='%CELL_VAL%' type='text' id='item2Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item3' name='item' value='batteryVoltage' data-item-number='3'>"
                                  "            <label for='item3'>3. Battery Voltage</label>"
                                  "            <input name='main_batt_voltage_pos' value='%BAT_VAL%' type='text' id='item3Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item4' name='item' value='craftName' data-item-number='4'>"
                                  "            <label for='item4'>4. Craft Name</label>"
                                  "            <input name='craft_name_pos' value='%CRAFT_VAL%' type='text' id='item4Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item5' name='item' value='satellites' data-item-number='5'>"
                                  "            <label for='item5'>5. Satellites</label>"
                                  "            <input name='gps_sats_pos' value='%SATS_VAL%' type='text' id='item5Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item6' name='item' value='homeArrow' data-item-number='6'>"
                                  "            <label for='item6'>6. Home Arrow</label>"
                                  "            <input name='home_dir_pos' value='%HOMEARROW_VAL%' type='text' id='item6Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item7' name='item' value='distanceFromHome' data-item-number='7'>"
                                  "            <label for='item7'>7. Distance from Home</label>"
                                  "            <input name='home_dist_pos' value='%DIST_VAL%' type='text' id='item7Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item8' name='item' value='groundSpeed' data-item-number='8'>"
                                  "            <label for='item8'>8. Ground Speed</label>"
                                  "            <input name='gps_speed_pos' value='%SPEED_VAL%' type='text' id='item8Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item9' name='item' value='latitude' data-item-number='9'>"
                                  "            <label for='item9'>9. Latitude</label>"
                                  "            <input name='gps_lat_pos' value='%LAT_VAL%' type='text' id='item9Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item10' name='item' value='longitude' data-item-number='10'>"
                                  "            <label for='item10'>10. Longitude</label>"
                                  "            <input name='gps_lon_pos' value='%LON_VAL%' type='text' id='item10Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "        <div class='item'>"
                                  "            <input type='radio' id='item11' name='item' value='longitude' data-item-number='11'>"
                                  "            <label for='item11'>11. Crosshair</label>"
                                  "            <input name='crosshairs_pos' value='%CROSS_VAL%' type='text' id='item11Field' class='readonly-field' readonly>"
                                  "        </div>"
                                  "		<button type='submit'>Save OSD Selections</button>"
                                  "		</form>"
                                  "    </div>"
                                  "    <h2>Select Item Location</h2>"
                                  "	<table>"
                                  "		<tbody>"
                                  "			<td data-value='234'>234</td>"
                                  "		</tbody>"
                                  "	<table>"
                                  "	* 234 will hide the OSD element."
                                  "	<table>"
                                  "        <tbody>"
                                  "            <script>"
                                  "                let start = 2048;"
                                  "                let increment = 1;"
                                  "                let additionalIncrement = 5;"
                                  "                let currentNumber = start;"
                                  "				let displayNumber = currentNumber;"
                                  "				let itemCount = 1;"
                                  "				const readOnlyFields = document.querySelectorAll('.readonly-field');"
                                  "                for (let i = 0; i < 16; i++) {"
                                  "                    document.write('<tr>');"
                                  "                    for (let j = 0; j < 27; j++) {"
                                  "						displayNumber = currentNumber;"
                                  "						for (let field of readOnlyFields) {"
                                  "							if (field.value == currentNumber) {"
                                  "								displayNumber = itemCount;"
                                  "								break;"
                                  "							}"
                                  "							itemCount++;"
                                  "						}"
                                  "						itemCount = 1;"
                                  "                        document.write('<td data-value=\' + currentNumber +  \'>' + displayNumber + '</td>');"
                                  "                        currentNumber += increment;"
                                  "                    }"
                                  "                    currentNumber += additionalIncrement;"
                                  "                    document.write('</tr>');"
                                  "                }"
                                  "				const tableCells = document.querySelectorAll('td');"
                                  "				let cellValue = 0;"
                                  "				for (let cellX of tableCells) {"
                                  "					cellValue = cellX.innerText;"
                                  "					if (cellValue <= 11) {"
                                  "						cellX.style.backgroundColor = getColorByNumber(cellValue);"
                                  "						readOnlyFields[(cellValue-1)].style.backgroundColor = cellX.style.backgroundColor;"
                                  "					}"
                                  "				}"
                                  "            </script>"
                                  "        </tbody>"
                                  "    </table>";
