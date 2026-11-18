"""Direct JPEG snapshots using Canon's WebView image.cgi interface."""

import base64
import http.client
import urllib.request


class HTTPCamera:
    def __init__(self, camera_ip, username="admin", password="", timeout=10):
        self.host = camera_ip
        self.url = "http://{}/-wvhttp-01-/image.cgi?cache=off".format(camera_ip)
        token = base64.b64encode((username + ":" + password).encode("utf-8")).decode("ascii")
        self.headers = {"Authorization": "Basic " + token, "Cache-Control": "no-cache"}
        self.connection = http.client.HTTPConnection(camera_ip, timeout=timeout)
        self.timeout = timeout
        self.digest_opener = None
        self.username = username
        self.password = password

    def capture(self):
        if self.digest_opener is not None:
            with self.digest_opener.open(self.url, timeout=self.timeout) as response:
                return self._check_image(response.read(16_000_001))
        self.connection.request("GET", "/-wvhttp-01-/image.cgi?cache=off", headers=self.headers)
        response = self.connection.getresponse()
        challenge = response.getheader("WWW-Authenticate", "")
        data = response.read(16_000_001)
        if response.status == 401 and "digest" in challenge.lower():
            self.connection.close()
            manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
            manager.add_password(None, "http://" + self.host + "/", self.username, self.password)
            self.digest_opener = urllib.request.build_opener(
                urllib.request.ProxyHandler({}), urllib.request.HTTPDigestAuthHandler(manager)
            )
            return self.capture()
        if response.status != 200:
            raise RuntimeError("Camera HTTP {} {}".format(response.status, response.reason))
        return self._check_image(data)

    @staticmethod
    def _check_image(data):
        if len(data) > 16_000_000:
            raise ValueError("Camera response exceeded the 16 MB image limit.")
        if not data.startswith(b"\xff\xd8"):
            raise ValueError("The camera did not return a JPEG image.")
        return data

    def close(self):
        self.connection.close()
