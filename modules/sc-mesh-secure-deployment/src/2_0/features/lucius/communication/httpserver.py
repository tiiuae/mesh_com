from http.server import BaseHTTPRequestHandler, HTTPServer

import threading

from application.types import ResponseCallback

PORT = 15884
NUM_CLIENTS = 2
SOCKET_TIMEOUT_SECONDS = 2

def makeHTTPRequestHandler(response_callback: ResponseCallback):
  class CustomHTTPRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
      self.response_callback = response_callback
      super(CustomHTTPRequestHandler, self).__init__(*args, **kwargs)

    def do_GET(self):
      self.send_response(200)
      self.send_header("Content-type", "application/json")
      self.end_headers()
      self.response_callback(None, lambda response: self.wfile.write(response.encode()))

  return CustomHTTPRequestHandler

class LuciusHTTPServer:
  def __init__(self, response_callback: ResponseCallback, host: str = '', port: int = PORT) -> None:
    self.httpd = HTTPServer((host, port), makeHTTPRequestHandler(response_callback))
    self.server_thread = threading.Thread(target=self.httpd.serve_forever)

  def start(self):
    self.server_thread.start()

  def stop(self):
    print("stopping http server")
    self.httpd.shutdown()
    print("stopping server thread")
    self.server_thread.join()