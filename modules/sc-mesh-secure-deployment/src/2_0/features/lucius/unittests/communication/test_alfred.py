from lucius.communication.alfred import AlfredComms, calculate_node_id

def test_calculate_node_id():
  print(calculate_node_id())
  assert isinstance(calculate_node_id(), str)

class TestAlfredComms:
  def test_parse_response(self):
    alfred_response = '{ "00:30:1a:4f:c8:21", "{ \"name\": \"test\" }" },'
    json_response = AlfredComms.parse_response(alfred_response)

    assert isinstance(json_response, list)
    assert len(json_response) == 1
    assert json_response[0]["mac"] == "00:30:1a:4f:c8:21"
    assert json_response[0]["data"]["name"] == "test"

  def test_parse_response_newline_at_end(self):
    alfred_response = '{ "00:30:1a:4f:c8:21", "{ \"name\": \"test\" }\x0a" },'
    json_response = AlfredComms.parse_response(alfred_response)

    assert isinstance(json_response, list)
    assert len(json_response) == 1
    assert json_response[0]["mac"] == "00:30:1a:4f:c8:21"
    assert json_response[0]["data"]["name"] == "test"

  def test_parse_response_multiple_lines(self):
    alfred_response = """{ "00:30:1a:4f:c8:21", "{ \"name\": \"test\" }" },
    { "00:30:1a:4f:c8:23", "{ \"name\": \"test_other\" }" },"""
    json_response = AlfredComms.parse_response(alfred_response)

    assert isinstance(json_response, list)
    assert len(json_response) == 2
    assert json_response[0]["mac"] == "00:30:1a:4f:c8:21"
    assert json_response[0]["data"]["name"] == "test"
    assert json_response[1]["mac"] == "00:30:1a:4f:c8:23"
    assert json_response[1]["data"]["name"] == "test_other"