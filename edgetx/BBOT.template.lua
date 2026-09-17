-- FEL BattleBot CRSF telemetry screen for EdgeTX 2.8.
-- This template is combined with TELEMETRY_FIELD_MAP during the build.

local FRAME_TYPE = 0x80
local SUBTYPE = 0xF3
local VERSION = 2
local HISTORY_SIZE = 64
local STALE_TICKS = 100 -- getTime() uses 10 ms ticks
local RAW_ROWS_PER_PAGE = 5

-- {{TELEMETRY_SCHEMA}}
local SCREEN = "{{TELEMETRY_SCREEN}}"

local fieldByName = {}
for index=1,#fields do
  fieldByName[fields[index].name] = fields[index]
end

local knownSensors = {
  rpm = { id=0xBB01, name="RPM", unit=UNIT_RPMS, precision=0 },
  accel1X = { id=0xBB02, name="A1X", unit=UNIT_G, precision=2 },
  accel1Y = { id=0xBB03, name="A1Y", unit=UNIT_G, precision=2 },
  accel1Z = { id=0xBB04, name="A1Z", unit=UNIT_G, precision=2 },
  accel2X = { id=0xBB05, name="A2X", unit=UNIT_G, precision=2 },
  accel2Y = { id=0xBB06, name="A2Y", unit=UNIT_G, precision=2 },
  accel2Z = { id=0xBB07, name="A2Z", unit=UNIT_G, precision=2 }
}

local axisDefinitions = {
  { key="accel1X", label="A1X" },
  { key="accel1Y", label="A1Y" },
  { key="accel1Z", label="A1Z" },
  { key="accel2X", label="A2X" },
  { key="accel2Y", label="A2Y" },
  { key="accel2Z", label="A2Z" }
}

local graphAxes = {}
for index=1,#axisDefinitions do
  local axis = axisDefinitions[index]
  if fieldByName[axis.key] ~= nil then graphAxes[#graphAxes + 1] = axis end
end

if FEL_BBOT_STATE == nil or FEL_BBOT_STATE.schemaKey ~= SCHEMA_KEY then
  local sharedHistory = { rpm={} }
  for index=1,#graphAxes do sharedHistory[graphAxes[index].key] = {} end

  FEL_BBOT_STATE = {
    schemaKey=SCHEMA_KEY,
    values={},
    history=sharedHistory,
    historyPosition=0,
    historyCount=0,
    packetCount=0,
    droppedCount=0,
    rejectedCount=0,
    lastSequence=nil,
    lastPacketTime=nil,
    lastInitTime=nil
  }
end

local state = FEL_BBOT_STATE
local selectedAxis = 1
local rawPage = 1

local function resetState()
  state.values = {}
  state.history = { rpm={} }
  for index=1,#graphAxes do state.history[graphAxes[index].key] = {} end

  state.historyPosition = 0
  state.historyCount = 0
  state.packetCount = 0
  state.droppedCount = 0
  state.rejectedCount = 0
  state.lastSequence = nil
  state.lastPacketTime = nil
  selectedAxis = 1
  rawPage = 1
end

local function hasBit(value, index)
  local bit = 2 ^ index
  return math.floor(value / bit) % 2 == 1
end

local function uint16be(data, index)
  return data[index] * 256 + data[index + 1]
end

local function int16be(data, index)
  local value = uint16be(data, index)
  if value >= 32768 then value = value - 65536 end
  return value
end

local function uint32be(data, index)
  return data[index] * 16777216 + data[index + 1] * 65536 + data[index + 2] * 256 + data[index + 3]
end

local function int32be(data, index)
  local value = uint32be(data, index)
  if value >= 2147483648 then value = value - 4294967296 end
  return value
end

local function float32be(data, index)
  local bits = uint32be(data, index)
  local sign = 1
  if bits >= 2147483648 then
    sign = -1
    bits = bits - 2147483648
  end

  local exponent = math.floor(bits / 8388608)
  local mantissa = bits % 8388608
  if exponent == 255 then
    if mantissa == 0 then return sign * math.huge end
    return 0 / 0
  end
  if exponent == 0 then return sign * (mantissa / 8388608) * 2 ^ -126 end
  return sign * (1 + mantissa / 8388608) * 2 ^ (exponent - 127)
end

local decoders = {
  int8_t = function(data, index)
    local value = data[index]
    if value >= 128 then value = value - 256 end
    return value
  end,
  uint8_t = function(data, index) return data[index] end,
  int16_t = int16be,
  uint16_t = uint16be,
  int32_t = int32be,
  uint32_t = uint32be,
  float = float32be
}

local function round(value)
  if value < 0 then return math.ceil(value - 0.5) end
  return math.floor(value + 0.5)
end

local function publishKnownField(name, value)
  local sensor = knownSensors[name]
  if sensor == nil or value == nil then return end

  local publishedValue = value
  if name ~= "rpm" then
    -- Configured acceleration is m/s^2; EdgeTX UNIT_G uses centi-g.
    publishedValue = round(value * 100 / 9.80665)
  end

  setTelemetryValue(sensor.id, 0, 0, round(publishedValue), sensor.unit, sensor.precision, sensor.name)
end

local function appendHistory()
  state.historyPosition = state.historyPosition % HISTORY_SIZE + 1
  if state.historyCount < HISTORY_SIZE then state.historyCount = state.historyCount + 1 end
  state.history.rpm[state.historyPosition] = state.values.rpm
  for index=1,#graphAxes do
    local key = graphAxes[index].key
    state.history[key][state.historyPosition] = state.values[key]
  end
end

local function acceptPacket(data)
  if #data ~= PAYLOAD_SIZE or data[1] ~= SUBTYPE or data[2] ~= VERSION then
    state.rejectedCount = state.rejectedCount + 1
    return
  end

  local sequence = data[3]
  if state.lastSequence ~= nil then
    local expected = (state.lastSequence + 1) % 256
    state.droppedCount = state.droppedCount + (sequence - expected) % 256
  end
  state.lastSequence = sequence

  local validFields = uint32be(data, 4)
  for index=1,#fields do
    local field = fields[index]
    if hasBit(validFields, index - 1) then
      state.values[field.name] = decoders[field.type](data, field.offset)
      publishKnownField(field.name, state.values[field.name])
    else
      state.values[field.name] = nil
    end
  end

  state.packetCount = state.packetCount + 1
  state.lastPacketTime = getTime()
  appendHistory()
end

local function drainFrames()
  while true do
    local command, data = crossfireTelemetryPop()
    if command == nil then return end
    if command == FRAME_TYPE then acceptPacket(data) end
  end
end

local function ageTicks()
  if state.lastPacketTime == nil then return nil end
  return getTime() - state.lastPacketTime
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
  if chronologicalIndex < 1 or chronologicalIndex > state.historyCount then return nil end
  local oldest = (state.historyPosition - state.historyCount) % HISTORY_SIZE + 1
  local index = (oldest + chronologicalIndex - 2) % HISTORY_SIZE + 1
  return series[index]
end

local function drawSeries(series, x, y, width, height, minimum, maximum, absolute)
  local previousX = nil
  local previousY = nil
  for point=1,state.historyCount do
    local value = historyAt(series, point)
    if value ~= nil then
      if absolute then value = math.abs(value) end
      value = clamp(value, minimum, maximum)
      local px = x
      if HISTORY_SIZE > 1 then px = x + math.floor((point - 1) * (width - 1) / (HISTORY_SIZE - 1)) end
      local py = y + height - 1 - math.floor((value - minimum) * (height - 1) / (maximum - minimum))
      if previousX ~= nil then lcd.drawLine(previousX, previousY, px, py, SOLID, 0) end
      previousX = px
      previousY = py
    else
      previousX = nil
      previousY = nil
    end
  end
end

local function formatAcceleration(value)
  if value == nil then return "--" end
  return string.format("%.3f", value)
end

local function formatInteger(value)
  if value == nil then return "--" end
  return string.format("%.0f", value)
end

local function drawGraph()
  if #graphAxes == 0 then
    drawHeader("BBOT GRAPH")
    lcd.drawText(LCD_W / 2, 28, "NO ACCEL FIELDS", SMLSIZE + CENTER)
    return
  end

  local axis = graphAxes[selectedAxis]
  drawHeader("BBOT " .. axis.label)

  lcd.drawRectangle(0, 9, LCD_W, 23)
  lcd.drawText(2, 10, "RPM", SMLSIZE)
  if state.values.rpm ~= nil then lcd.drawNumber(LCD_W - 2, 10, state.values.rpm, SMLSIZE + RIGHT) end
  drawSeries(state.history.rpm, 1, 10, LCD_W - 2, 21, 0, 4000, true)

  lcd.drawRectangle(0, 37, LCD_W, 27)
  lcd.drawText(2, 38, axis.label, SMLSIZE)
  lcd.drawLine(1, 50, LCD_W - 2, 50, DOTTED, 0)
  local current = state.values[axis.key]
  if current ~= nil then
    lcd.drawText(LCD_W - 2, 38, formatAcceleration(current) .. "m/s2", SMLSIZE + RIGHT)
  end
  drawSeries(state.history[axis.key], 1, 38, LCD_W - 2, 25, -1000, 1000, false)
end

local function drawFormattedValues()
  drawHeader("BBOT VALUES")
  local modeNames = { [1]="IDLE", [2]="FWD", [3]="SPIN" }
  local modeText = state.values.mode == nil and "--" or modeNames[state.values.mode] or formatInteger(state.values.mode)
  lcd.drawText(0, 10, "MODE " .. modeText, SMLSIZE)
  lcd.drawText(LCD_W, 10, "RPM " .. formatInteger(state.values.rpm), SMLSIZE + RIGHT)
  lcd.drawText(0, 22,
    "A1 " .. formatAcceleration(state.values.accel1X) .. " " ..
    formatAcceleration(state.values.accel1Y) .. " " .. formatAcceleration(state.values.accel1Z), SMLSIZE)
  lcd.drawText(0, 34,
    "A2 " .. formatAcceleration(state.values.accel2X) .. " " ..
    formatAcceleration(state.values.accel2Y) .. " " .. formatAcceleration(state.values.accel2Z), SMLSIZE)
  lcd.drawText(0, 46, "RX " .. state.packetCount .. " DROP " .. state.droppedCount, SMLSIZE)
  lcd.drawText(0, 56, "REJECT " .. state.rejectedCount, SMLSIZE)
  lcd.drawText(LCD_W, 56, "m/s2", SMLSIZE + RIGHT)
end

local function rawPageCount()
  return math.max(1, math.ceil(#fields / RAW_ROWS_PER_PAGE))
end

local function formatRawValue(field, value)
  if value == nil then return "--" end
  if field.type == "float" then return string.format("%.6g", value) end
  return formatInteger(value)
end

local function drawRawValues()
  local pageCount = rawPageCount()
  drawHeader("RAW " .. rawPage .. "/" .. pageCount)
  local firstField = (rawPage - 1) * RAW_ROWS_PER_PAGE + 1

  for row=0,RAW_ROWS_PER_PAGE - 1 do
    local index = firstField + row
    local field = fields[index]
    if field ~= nil then
      local y = 11 + row * 10
      lcd.drawText(0, y, field.name, SMLSIZE)
      lcd.drawText(LCD_W, y, formatRawValue(field, state.values[field.name]), SMLSIZE + RIGHT)
    end
  end
end

local function handleEvent(screen, event)
  if screen == "graph" and #graphAxes > 0 then
    if event == EVT_VIRTUAL_NEXT or event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK then
      selectedAxis = selectedAxis % #graphAxes + 1
    elseif event == EVT_VIRTUAL_PREV or event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK then
      selectedAxis = (selectedAxis - 2) % #graphAxes + 1
    end
  elseif screen == "raw" then
    if event == EVT_VIRTUAL_NEXT or event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK then
      rawPage = rawPage % rawPageCount() + 1
    elseif event == EVT_VIRTUAL_PREV or event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK then
      rawPage = (rawPage - 2) % rawPageCount() + 1
    end
  end
end

local function drawScreen(screen)
  if screen == "graph" then
    drawGraph()
  elseif screen == "values" then
    drawFormattedValues()
  elseif screen == "raw" then
    drawRawValues()
  else
    drawHeader("BBOT ERROR")
    lcd.drawText(0, 12, "UNKNOWN SCREEN", SMLSIZE)
  end
end

local function init()
  local currentTime = getTime()
  if state.lastInitTime == nil then
    resetState()
  else
    local timeSinceLastInit = currentTime - state.lastInitTime
    if timeSinceLastInit < 0 or timeSinceLastInit > 10 then resetState() end
  end
  state.lastInitTime = currentTime
  -- Calling pop once creates EdgeTX's shared Crossfire Lua receive queue.
  drainFrames()
end

local function background()
  drainFrames()
end

local function run(event)
  drainFrames()
  handleEvent(SCREEN, event)

  lcd.clear()
  drawScreen(SCREEN)
  return 0
end

return { init=init, background=background, run=run }
