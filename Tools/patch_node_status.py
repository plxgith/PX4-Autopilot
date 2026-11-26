from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

COMMON_XML 	= ROOT / "src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml"
SIZE_FILE 	= ROOT / "src/modules/mavlink/mavlink/pymavlink/tools/mavlink_messages_size.py"

MESSAGE_XML = """  <message id="502" name="NODE_STATUS">
      <description>General status information of an UAVCAN node. This NODE_STATUS is used to only display/log battery node status with ID 42 </description>
      <field type="uint64_t" name="timestamp" units="us">Timestamp (UNIX Epoch time or time since system boot).</field>
      <field type="uint32_t" name="uptime_sec" units="s">Time since the start-up of the node.</field>
      <field type="uint8_t" name="health" enum="UAVCAN_NODE_HEALTH">Generalized node health status.</field>
      <field type="uint8_t" name="mode" enum="UAVCAN_NODE_MODE">Generalized operating mode.</field>
      <field type="uint16_t" name="vendor_specific_status_code">Vendor-specific status information.</field>
    </message>"""

# The big whitespace is here only for formating reasons to fit properly in the file its being written in
SIZE_ENTRY 	= "(			   'NODE_STATUS' ,  24), # ID#502"

def patch_common_xml():
	text = COMMON_XML.read_text()

	if 'name="NODE_STATUS"' in text:
		return

	marker = "</messages>"

	idx = text.rfind(marker)

	if idx < 0:
		raise RuntimeError("xxx ERROR! Check the file src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml as there is a problem and node_status couldn't be added.")

	text = (
		text[:idx]
		+ MESSAGE_XML
		+ '\n'
		+ "  "		# The whitespace added here is for formating reasons
		+ text[idx:]
	)

	COMMON_XML.write_text(text)
	print("Patched common.xml - node_status")

def patch_mavlink_messages_size_py():
	text = SIZE_FILE.read_text()

	if "ID#502" in text:
		return

	# The whitespace added here for the format that is being used in the file we are reading from
	marker = "(                       'CIRCUIT_STATUS' ,  24)"

	idx = text.rfind(marker)

	if idx < 0:
		raise RuntimeError("xxx ERROR! Check the file src/modules/mavlink/mavlink/pymavlink/tools/mavlink_messages_size.py as there is a problem and node_status couldn't be added.")

	text = (
		text[:idx]
		+ SIZE_ENTRY
		+ '\n'
		+ text[idx:]
	)

	SIZE_FILE.write_text(text)
	print("Patched mavlink_messages_size.py - node_status")

# Call each patch function
patch_common_xml()
patch_mavlink_messages_size_py()
