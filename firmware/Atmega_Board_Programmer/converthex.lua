-- Do MD5 sumcheck of Arduino bootloader .hex file
-- Nick Gammon
-- 5 May 2012

require "getlines"

loader = nil
adder = 0

-- given a start address, deduce where the bootloader ends
end_addresses = {
  [0x1C00] = 0x2000,
  [0x1D00] = 0x2000,
  [0x1E00] = 0x2000,
  [0x3800] = 0x4000,
  [0x3E00] = 0x4000,
  [0x7000] = 0x8000,
  [0x7800] = 0x8000,
  [0x7E00] = 0x8000,
  [0xF800] = 0x10000,
  [0x1F000] = 0x20000,
  [0x1FC00] = 0x20000,
  [0x3E000] = 0x40000,
  }

function process (size, address, rectype, data)

  size = tonumber (size, 16)
  address = tonumber (address, 16) + adder
  rectype = tonumber (rectype)
  local binarydata = utils.fromhex (data)

  if rectype == 2 then  -- Extended Segment Address Record
    adder = tonumber (data, 16) * 16  -- high order address byte    
  elseif rectype == 0 then -- data record
    if loader == nil then
      start_address = address
      end_address = end_addresses [address]
      if end_address == nil then
        ColourNote ("red", "", "Don't know end address for " .. bit.tostring (address, 16))
        ColourNote ("red", "", "Please add to table: end_addresses")
        error "Cannot continue"
      end -- if end address not found

      -- work out loader length
      length = end_address - address
      -- pre-fill with 0xFF in case not every byte supplied
      loader = string.rep ("\255", length)
      print (string.format ("// Loader start: %X, length: %i", address, length))
    end -- no loader yet

    -- insert data over where the 0xFF was
    if address >= start_address and (address + size) <= end_address then
      loader = loader:sub (1, address - start_address) .. 
               binarydata .. 
               loader:sub (address - start_address + size + 1, length)
    else
      ColourNote ("red", "", "Address " .. bit.tostring (address, 16) .. " out of expected range.")
    end  -- if in range
  end -- if

end -- function process

print (string.rep ("-", 60))

-- get bootloader file
filename = utils.filepicker ("Choose a bootloader", nil, "hex", { hex = "Hex files" })

-- none chosen, give up
if not filename then return end

-- show file
local fn = string.match (filename, "\\([^\\]+)$")
print ("// File = ", fn)

-- process each line
for line in io.lines (filename) do
  size, address, rectype, data = string.match (line, "^:(%x%x)(%x%x%x%x)(%x%x)(%x+)%s*$")
  if size then 
    process (size, address, rectype, data:sub (1, -3))
  else
    ColourNote ("red", "", "Discarded line: " .. line)
  end -- if
end -- for loop

--print (utils.tohex (loader))

-- sumcheck it
Tell ("// MD5 sum = ")
md5sum = utils.tohex (utils.md5 (loader))
print ((string.gsub (md5sum, "(%x%x)", "%1 ")))

print ""

--     show bootloader in hex

-- convert into C array
print (string.format ("byte PROGMEM %s [] = {",
       string.gsub (fn, "[^%a%d]", "_")))
for i = 1, #loader do
  Tell (string.format ("0x%02X, ", loader:sub (i, i):byte ()))
  if (i - 1) % 16 == 15 then print "" end
end -- of for each byte
print (string.format ("}; // end of %s", string.gsub (fn, "[^%a%d]", "_")))