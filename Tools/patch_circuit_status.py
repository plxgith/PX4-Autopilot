from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

COMMON_XML 	= ROOT / "src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml"
SIZE_FILE 	= ROOT / "src/modules/mavlink/mavlink/pymavlink/tools/mavlink_messages_size.py"

MESSAGE_XML = """  <message id="501" name="CIRCUIT_STATUS">
      <description>Generic electrical circuit info.</description>
      <field type="uint64_t" name="timestamp" units="ms">Timestamp (time since system boot).</field>
      <field type="uint8_t" name="id" instance="true">Circuit Status ID</field>
      <field type="float" name="voltage" units="V" invalid="NaN">Voltage, NaN if unknown</field>
      <field type="float" name="current" units="A" invalid="NaN">Current draw, NaN if unknown</field>
      <!-- <field type="uint8_t" name="flags" enum="MAV_CIRCUIT_STATUS" display="bitmask">Status flags (bitmask)</field> -->
    </message>"""

# The big whitespace is here only for formating reasons to fit properly in the file its being written in
SIZE_ENTRY 	= "(			   'CIRCUIT_STATUS' ,  24), # ID#501"

def patch_common_xml():
	text = COMMON_XML.read_text()

	if 'name="CIRCUIT_STATUS"' in text:
		return

	marker = "</messages>"

	idx = text.rfind(marker)

	if idx < 0:
		raise RuntimeError("xxx ERROR! Check the file src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml as there is a problem and circuit_status couldn't be added.")

	text = (
		text[:idx]
		+ MESSAGE_XML
		+ '\n'
		+ "  "		# The whitespace added here is for formating reasons
		+ text[idx:]
	)

	COMMON_XML.write_text(text)
	print("Patched common.xml")

def patch_mavlink_messages_size_py():
	text = SIZE_FILE.read_text()

	if "ID#501" in text:
		return

	# The whitespace added here for the format that is being used in the file we are reading from
	marker = "(                       'AUTOPILOT_VERSION' ,  68)"

	idx = text.rfind(marker)

	if idx < 0:
		raise RuntimeError("xxx ERROR! Check the file src/modules/mavlink/mavlink/pymavlink/tools/mavlink_messages_size.py as there is a problem and circuit_status couldn't be added.")

	text = (
		text[:idx]
		+ SIZE_ENTRY
		+ '\n'
		+ text[idx:]
	)

	SIZE_FILE.write_text(text)
	print("Patched mavlink_messages_size.py")

# Call each patch function
patch_common_xml()
patch_mavlink_messages_size_py()
