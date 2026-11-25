"""Direct JPEG snapshots using Canon's WebView image.cgi interface."""

import base64
import http.client
import urllib.request


class HTTPCamera:
    def __init__(self, camera_ip, username="admin", password="", timeout=10):
        self.host = camera_ip
        self.url = "http://{}/-wvhttp-01-/image.cgi?cache=off".format(camera_ip)
        self.headers = {"Cache-Control": "no-cache"}
        self.connection = http.client.HTTPConnection(camera_ip, timeout=timeout)
        self.timeout = timeout
        self.digest_opener = None
        self.username = username
        self.password = password

    def _reset_connection(self):
        try:
            self.connection.close()
        except Exception:
            pass
        self.connection = http.client.HTTPConnection(self.host, timeout=self.timeout)

    def _request(self):
        retryable = (
            http.client.RemoteDisconnected,
            http.client.BadStatusLine,
            http.client.CannotSendRequest,
            http.client.ResponseNotReady,
            ConnectionResetError,
            ConnectionAbortedError,
            BrokenPipeError,
        )
        for attempt in range(2):
            try:
                self.connection.request("GET", "/-wvhttp-01-/image.cgi?cache=off", headers=self.headers)
                return self.connection.getresponse()
            except retryable:
                self._reset_connection()
                if attempt > 0:
                    raise
            except BaseException:
                self._reset_connection()
                raise

    def capture(self):
        if self.digest_opener is not None:
            request = urllib.request.Request(self.url, headers={"Cache-Control": "no-cache"})
            with self.digest_opener.open(request, timeout=self.timeout) as response:
                return self._check_image(response.read(16_000_001))
        response = self._request()
        try:
            challenge = response.getheader("WWW-Authenticate", "")
            status, reason = response.status, response.reason
            data = response.read(16_000_001)
            if len(data) > 16_000_000:
                self._reset_connection()
        except BaseException:
            self._reset_connection()
            raise
        finally:
            try:
                response.close()
            except Exception:
                pass
        if status == 401:
            challenge_lower = challenge.lower()
            if "digest" in challenge_lower:
                self.connection.close()
                manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
                manager.add_password(None, "http://" + self.host + "/", self.username, self.password)
                self.digest_opener = urllib.request.build_opener(
                    urllib.request.ProxyHandler({}), urllib.request.HTTPDigestAuthHandler(manager)
                )
                return self.capture()
            if "basic" in challenge_lower and "Authorization" not in self.headers:
                token = base64.b64encode((self.username + ":" + self.password).encode("utf-8")).decode("ascii")
                self.headers["Authorization"] = "Basic " + token
                return self.capture()
        if status != 200:
            raise RuntimeError("Camera HTTP {} {}".format(status, reason))
        return self._check_image(data)

    @staticmethod
    def _check_image(data):
        if len(data) > 16_000_000:
            raise ValueError("Camera response exceeded the 16 MB image limit.")
        if not data.startswith(b"\xff\xd8"):
            raise ValueError("The camera did not return a JPEG image.")
        if not data.rstrip(b"\x00\r\n").endswith(b"\xff\xd9"):
            raise ValueError("The camera returned a truncated JPEG image.")
        return data

    def close(self):
        self.connection.close()
