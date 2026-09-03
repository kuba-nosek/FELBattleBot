-- FEL BattleBot custom CRSF telemetry screen for EdgeTX 2.8.
-- Copy this file to /SCRIPTS/TELEMETRY/BBOT.lua on the radio SD card.
-- Wire protocol: CRSF 0x80, private subtype 0xF3, version 1.

local FRAME_TYPE = 0x80
local SUBTYPE = 0xF3
local VERSION = 1
local PAYLOAD_SIZE = 22
local HISTORY_SIZE = 64
local STALE_TICKS = 100 -- getTime() uses 10 ms ticks

local VALID_RPM = 0x01
local VALID_A1X = 0x02
local VALID_A1Y = 0x04
local VALID_A1Z = 0x08
local VALID_A2X = 0x10
local VALID_A2Y = 0x20
local VALID_A2Z = 0x40

local sensors = {
  { id=0xBB01, name="RPM", unit=UNIT_RPMS, precision=0 },
  { id=0xBB02, name="A1X", unit=UNIT_G, precision=2 },
  { id=0xBB03, name="A1Y", unit=UNIT_G, precision=2 },
  { id=0xBB04, name="A1Z", unit=UNIT_G, precision=2 },
  { id=0xBB05, name="A2X", unit=UNIT_G, precision=2 },
  { id=0xBB06, name="A2Y", unit=UNIT_G, precision=2 },
  { id=0xBB07, name="A2Z", unit=UNIT_G, precision=2 },
  { id=0xBB08, name="P1G", unit=UNIT_G, precision=2 },
  { id=0xBB09, name="P2G", unit=UNIT_G, precision=2 }
}

local axes = {
  { key="a1x", label="A1X", bit=VALID_A1X, sensor=2 },
  { key="a1y", label="A1Y", bit=VALID_A1Y, sensor=3 },
  { key="a1z", label="A1Z", bit=VALID_A1Z, sensor=4 },
  { key="a2x", label="A2X", bit=VALID_A2X, sensor=5 },
  { key="a2y", label="A2Y", bit=VALID_A2Y, sensor=6 },
  { key="a2z", label="A2Z", bit=VALID_A2Z, sensor=7 }
}

local values = {
  rpm=nil, a1x=nil, a1y=nil, a1z=nil,
  a2x=nil, a2y=nil, a2z=nil, p1g=nil, p2g=nil
}
local history = { rpm={}, a1x={}, a1y={}, a1z={}, a2x={}, a2y={}, a2z={} }
local historyPosition = 0
local historyCount = 0
local selectedAxis = 1
local graphMode = true
local suppressEnterBreak = false
local packetCount = 0
local droppedCount = 0
local rejectedCount = 0
local lastSequence = nil
local lastPacketTime = nil

local function hasBit(value, bit)
  return value % (bit * 2) >= bit
end

local function uint16be(data, index)
  return data[index] * 256 + data[index + 1]
end

local function int16be(data, index)
  local value = uint16be(data, index)
  if value >= 32768 then value = value - 65536 end
  return value
end

local function publish(sensorIndex, value)
  local sensor = sensors[sensorIndex]
  setTelemetryValue(
    sensor.id, 0, 0, value, sensor.unit, sensor.precision, sensor.name)
end

local function appendHistory()
  historyPosition = historyPosition % HISTORY_SIZE + 1
  if historyCount < HISTORY_SIZE then historyCount = historyCount + 1 end
  history.rpm[historyPosition] = values.rpm
  for index=1,#axes do
    local key = axes[index].key
    history[key][historyPosition] = values[key]
  end
end

local function acceptPacket(data)
  if #data ~= PAYLOAD_SIZE or data[1] ~= SUBTYPE or data[2] ~= VERSION then
    rejectedCount = rejectedCount + 1
    return
  end

  local sequence = data[3]
  if lastSequence ~= nil then
    local expected = (lastSequence + 1) % 256
    droppedCount = droppedCount + (sequence - expected) % 256
  end
  lastSequence = sequence

  local valid = data[4]
  local decoded = {
    int16be(data, 5), int16be(data, 7), int16be(data, 9),
    int16be(data, 11), int16be(data, 13), int16be(data, 15),
    int16be(data, 17), uint16be(data, 19), uint16be(data, 21)
  }

  if hasBit(valid, VALID_RPM) then
    values.rpm = decoded[1]
    publish(1, decoded[1])
  else
    values.rpm = nil
  end

  for index=1,#axes do
    local axis = axes[index]
    if hasBit(valid, axis.bit) then
      values[axis.key] = decoded[index + 1]
      publish(axis.sensor, decoded[index + 1])
    else
      values[axis.key] = nil
    end
  end

  if hasBit(valid, VALID_A1X) or hasBit(valid, VALID_A1Y) or
     hasBit(valid, VALID_A1Z) then
    values.p1g = decoded[8]
    publish(8, decoded[8])
  else
    values.p1g = nil
  end
  if hasBit(valid, VALID_A2X) or hasBit(valid, VALID_A2Y) or
     hasBit(valid, VALID_A2Z) then
    values.p2g = decoded[9]
    publish(9, decoded[9])
  else
    values.p2g = nil
  end

  packetCount = packetCount + 1
  lastPacketTime = getTime()
  appendHistory()
end

local function drainFrames()
  while true do
    local command, data = crossfireTelemetryPop()
    if command == nil then return end
    if command == FRAME_TYPE then acceptPacket(data) end
  end
end

local function formatCentiG(value)
  if value == nil then return "--" end
  return string.format("%.2f", value / 100)
end

local function ageTicks()
  if lastPacketTime == nil then return nil end
  return getTime() - lastPacketTime
end

local function drawHeader(title)
  local age = ageTicks()
  lcd.drawText(0, 0, title, SMLSIZE)
  if age == nil then
    lcd.drawText(LCD_W, 0, "NO DATA", SMLSIZE + RIGHT + BLINK)
  elseif age > STALE_TICKS then
    lcd.drawText(LCD_W, 0, "STALE", SMLSIZE + RIGHT + BLINK)
  else
    lcd.drawText(LCD_W, 0, "LIVE", SMLSIZE + RIGHT)
  end
end

local function clamp(value, minimum, maximum)
  if value < minimum then return minimum end
  if value > maximum then return maximum end
  return value
end

local function historyAt(series, chronologicalIndex)
  if chronologicalIndex < 1 or chronologicalIndex > historyCount then return nil end
  local oldest = (historyPosition - historyCount) % HISTORY_SIZE + 1
  local index = (oldest + chronologicalIndex - 2) % HISTORY_SIZE + 1
  return series[index]
end

local function drawSeries(series, x, y, width, height, minimum, maximum, absolute)
  local previousX = nil
  local previousY = nil
  for point=1,historyCount do
    local value = historyAt(series, point)
    if value ~= nil then
      if absolute then value = math.abs(value) end
      value = clamp(value, minimum, maximum)
      local px = x
      if HISTORY_SIZE > 1 then
        px = x + math.floor((point - 1) * (width - 1) / (HISTORY_SIZE - 1))
      end
      local py = y + height - 1 -
        math.floor((value - minimum) * (height - 1) / (maximum - minimum))
      if previousX ~= nil then
        lcd.drawLine(previousX, previousY, px, py, SOLID, 0)
      end
      previousX = px
      previousY = py
    else
      previousX = nil
      previousY = nil
    end
  end
end

local function drawGraph()
  local axis = axes[selectedAxis]
  drawHeader("BBOT " .. axis.label)

  lcd.drawRectangle(0, 9, LCD_W, 23)
  lcd.drawText(2, 10, "RPM", SMLSIZE)
  if values.rpm ~= nil then
    lcd.drawNumber(LCD_W - 2, 10, values.rpm, SMLSIZE + RIGHT)
  end
  drawSeries(history.rpm, 1, 10, LCD_W - 2, 21, 0, 4000, true)

  lcd.drawRectangle(0, 37, LCD_W, 27)
  lcd.drawText(2, 38, axis.label, SMLSIZE)
  lcd.drawLine(1, 50, LCD_W - 2, 50, DOTTED, 0)
  local current = values[axis.key]
  if current ~= nil then
    lcd.drawText(LCD_W - 2, 38, formatCentiG(current) .. "g", SMLSIZE + RIGHT)
  end
  drawSeries(history[axis.key], 1, 38, LCD_W - 2, 25, -10000, 10000, false)
end

local function drawList()
  drawHeader("BBOT VALUES")
  local rpmText = values.rpm == nil and "--" or tostring(values.rpm)
  lcd.drawText(0, 10, "RPM " .. rpmText, SMLSIZE)
  lcd.drawText(LCD_W, 10,
    "PK " .. formatCentiG(values.p1g) .. "/" .. formatCentiG(values.p2g),
    SMLSIZE + RIGHT)
  lcd.drawText(0, 20,
    "A1 " .. formatCentiG(values.a1x) .. " " ..
    formatCentiG(values.a1y) .. " " .. formatCentiG(values.a1z), SMLSIZE)
  lcd.drawText(0, 30,
    "A2 " .. formatCentiG(values.a2x) .. " " ..
    formatCentiG(values.a2y) .. " " .. formatCentiG(values.a2z), SMLSIZE)
  lcd.drawText(0, 42,
    "RX " .. packetCount .. " DROP " .. droppedCount, SMLSIZE)
  lcd.drawText(0, 52, "REJECT " .. rejectedCount, SMLSIZE)
  lcd.drawText(LCD_W, 52, "g avg / PK max", SMLSIZE + RIGHT)
end

local function init()
  -- Calling pop once creates EdgeTX's Crossfire Lua receive queue. Frames are
  -- then collected even while this screen is running in the background.
  drainFrames()
end

local function background()
  drainFrames()
end

local function run(event)
  drainFrames()

  if event == EVT_ENTER_LONG then
    graphMode = not graphMode
    suppressEnterBreak = true
  elseif event == EVT_ENTER_BREAK then
    if suppressEnterBreak then
      suppressEnterBreak = false
    else
      selectedAxis = selectedAxis % #axes + 1
    end
  end

  lcd.clear()
  if graphMode then drawGraph() else drawList() end
  return 0
end

return { init=init, background=background, run=run }
