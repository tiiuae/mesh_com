# REF: https://github.com/cedric-dufour/scriptisms/blob/master/misc/m2requests.py
import os
import os.path
import urllib.parse

import M2Crypto.Engine
import M2Crypto.httpslib
import M2Crypto.m2
import M2Crypto.SSL
import requests.adapters
import requests.cookies
import requests.models
import requests.structures
import requests.utils
import urllib3.response

# Parameters
M2REQUESTS_VERSION='1.0.20200130a'
M2REQUESTS_SSL_ENGINE_ID = os.getenv("M2REQUESTS_SSL_ENGINE_ID", "pkcs11")
M2REQUESTS_SSL_ENGINE_PATH = os.getenv(
    "M2REQUESTS_SSL_ENGINE_PATH",
    "/usr/lib/x86_64-linux-gnu/engines-3/libpkcs11.so",
)
M2REQUESTS_SSL_MODULE_PATH = os.getenv(
    "M2REQUESTS_SSL_MODULE_PATH", "/usr/lib/softhsm/libsofthsm2.so"
)
M2REQUESTS_SSL_CONTEXT = os.getenv("M2REQUESTS_SSL_CONTEXT", "tls")
M2REQUESTS_SSL_CIPHERS = os.getenv(
    "M2REQUESTS_SSL_CIPHERS",
    "EECDH+AESGCM:EECDH+aECDSA:EECDH+aRSA:EDH+AESGCM:EDH+aECDSA:EDH+aRSA:!SHA1:!SHA256:!SHA384:!MEDIUM:!LOW:!EXP:!aNULL:!eNULL:!PSK:!SRP:@STRENGTH",
)
M2REQUESTS_SSL_VERIFY = bool(os.getenv("M2REQUESTS_SSL_VERIFY", True))
M2REQUESTS_SSL_VERIFY_DEPTH = int(os.getenv("M2REQUESTS_SSL_VERIFY_DEPTH", 1))
M2REQUESTS_SSL_CA_PATH = os.getenv("M2REQUESTS_SSL_CA_PATH", None)
M2REQUESTS_SSL_CERT_PATH = os.getenv("M2REQUESTS_SSL_CERT_PATH", "pkcs11:token=rsa")
M2REQUESTS_SSL_KEY_PATH = os.getenv("M2REQUESTS_SSL_KEY_PATH", "pkcs11:token=rsa")


class M2HttpsAdapter(requests.adapters.BaseAdapter):
    """
    M2Crypto-based HTTPS adapter for Python requests Session mount().

    This adapter leverages M2Crypto ability to hook an external SSL engine into
    the SSL context used when establishing the HTTPS connection.

    As opposed to stock Python requests Session adapters, this adapter does
    *not* rely on a connections pool. A new connection is established for each
    call to send(), using the *same* SSL context(/engine), which can *not* be
    changed once initialized.

    This adapter currently does *not* support proxied connections, nor HTTP
    streams.

    USAGE:
      from m2requests import M2HttpsAdapter
      from requests import Session
      request = Session()
      request.mount("https://", M2HttpsAdapter())
      request.verify="path/to/ca-bundle.pem"
      # PKCS#11 URI; REF: https://tools.ietf.org/html/rfc7512
      request.cert=("pkcs11:type=cert;...", "pkcs11:type=private;...")
      request.get("https://...")

    REF: https://github.com/psf/requests/blob/main/src/requests/adapters.py
    """

    def __init__(self):
        super(M2HttpsAdapter, self).__init__()

        # SSL engine
        self._ssl_engine = None
        self.ssl_engine_id = None
        self.ssl_engine_path = None
        self.ssl_engine_module_path = None

        # SSL context
        self._ssl_context = None
        self.ssl_context = None
        self.ssl_context_ciphers = None
        self.ssl_context_verify = None
        self.ssl_context_verify_depth = None
        self.ssl_context_ca_path = None
        self.ssl_context_cert_path = None
        self.ssl_context_key_path = None

    #
    # BaseAdapter methods (implement)
    #

    def send(
        self,
        request,
        stream=False,
        timeout=None,
        verify=True,
        cert=None,
        proxies=None,
    ):

        # Implementation checks
        if stream:
            raise RuntimeError("Streamed requests are not implemented")
        if proxies:
            raise RuntimeError("Proxied requests are not implemented")

        # Server verification (<-> certification authority)
        ca_path = None
        if verify:
            if isinstance(verify, bool):
                ca_path = ""  # system CA bundle
            else:
                ca_path = verify
                verify = True

        # Client authentication (<-> certificate and private key)
        cert_path = None
        key_path = None
        if cert is not None:
            try:
                (cert_path, key_path) = cert
            except ValueError:
                raise ValueError(
                    "Please specify certificate and private key separately"
                )

        # Parse URL
        url_parts = urllib.parse.urlparse(request.url)
        url_selector = url_parts.path
        if url_parts.query:
            url_selector += "?" + url_parts.query

        # HTTPS connection
        # NOTE: httplib (Py2) = http.client (Py3)
        self.init_ssl_context(
            verify=verify,
            ca_path=ca_path,
            cert_path=cert_path,
            key_path=key_path,
        )
        # REF: https://docs.python.org/3/library/http.client.html#http.client.HTTPConnection
        # NOTE: Unfortunately, underlying http_client.HTTPConnection timeout
        #       parameter is not passed through by M2Crypto HTTPSConnection
        conn = M2Crypto.httpslib.HTTPSConnection(
            host=url_parts.netloc, ssl_context=self._ssl_context
        )

        # Send request
        conn.request(
            method=request.method,
            url=url_selector,
            body=request.body,
            headers=request.headers,
        )

        # Get response
        # REF: https://docs.python.org/3/library/http.client.html#http.client.HTTPResponse
        # REF: https://docs.python.org/3/library/http.client.html#http.client.HTTPConnection.getresponse
        resp = conn.getresponse()

        # Build output response
        # REF: https://github.com/psf/requests/blob/main/src/requests/adapters.py
        output = requests.models.Response()
        output.connection = self
        output.request = request
        output.url = url_parts.geturl()
        output.status_code = getattr(resp, "status", None)
        output.reason = getattr(resp, "reason", None)
        output.headers = requests.structures.CaseInsensitiveDict(
            getattr(resp, "headers", {})
        )
        output.encoding = requests.utils.get_encoding_from_headers(
            output.headers
        )
        requests.cookies.extract_cookies_to_jar(output.cookies, request, resp)
        output.raw = resp

        return output

    def close(self):
        pass

    #
    # self methods
    #

    def init_ssl_engine(
        self,
        id=M2REQUESTS_SSL_ENGINE_ID,
        engine_path=M2REQUESTS_SSL_ENGINE_PATH,
        module_path=M2REQUESTS_SSL_MODULE_PATH,
    ):
        """
        Initialize (load) the given SSL engine.

        An SSL engine can only be initialized once and can *not* be changed
        afterwards.

        @param  str  id           SSL engine ID
        @param  str  engine_path  SSL engine's library path
        @param  str  module_path  SSL engine's module library path
        @throw  ValueError        Invalid argument value
        @throw  RuntimeError      Invalid argument (changing internal state)
        """

        # Validation
        # ... ID
        if not id:
            raise ValueError("Missing SSL engine ID (id)")
        if self.ssl_engine_id is not None and self.ssl_engine_id != id:
            raise RuntimeError(
                f"SSL engine already initialized with different ID ({self.ssl_engine_id})"
            )
        # ... engine path
        if not engine_path:
            raise ValueError("Missing SSL engine library path (engine_path)")
        if (
            self.ssl_engine_path is not None
            and self.ssl_engine_path != engine_path
        ):
            raise RuntimeError(
                f"SSL engine already initialized with different engine ({self.ssl_engine_path})"
            )
        # ... module path
        if not module_path:
            raise ValueError(
                "Missing SSL engine module library path (module_path)"
            )
        if (
            self.ssl_engine_module_path is not None
            and self.ssl_engine_module_path != module_path
        ):
            raise RuntimeError(
                f"SSL engine already initialized with different engine ({self.ssl_engine_module_path})"
            )

        # Load the SSL engine
        if self._ssl_engine is not None:
            return
        M2Crypto.Engine.load_dynamic_engine(id, engine_path)
        ssl_engine = M2Crypto.Engine.Engine(id)
        ssl_engine.ctrl_cmd_string("MODULE_PATH", module_path)
        M2Crypto.m2.engine_init(M2Crypto.m2.engine_by_id(id))

        # Save SSL engine settings
        self._ssl_engine = ssl_engine
        self.ssl_engine_id = id
        self.ssl_engine_path = engine_path
        self.ssl_engine_module_path = module_path

    def init_ssl_context(
        self,
        context=M2REQUESTS_SSL_CONTEXT,
        ciphers=M2REQUESTS_SSL_CIPHERS,
        verify=M2REQUESTS_SSL_VERIFY,
        verify_depth=M2REQUESTS_SSL_VERIFY_DEPTH,
        ca_path=M2REQUESTS_SSL_CA_PATH,
        cert_path=M2REQUESTS_SSL_CERT_PATH,
        key_path=M2REQUESTS_SSL_KEY_PATH,
    ):
        """
        Initialize (preload) the given SSL context.

        An SSL context can only be initialized once and can *not* be changed
        afterwards. If the given certificate/private key path looks like a
        PKCS#11 RFC7512 URI ('pkcs11:...'), the SSL engine ID-ed 'pkcs11' will
        be automatically initialized.

        @param  str   context       SSL context (M2Crypto stanza)
        @param  str   ciphers       SSL ciphers
        @param  bool  verify        SSL server verification flag
        @param  int   verify_depth  SSL server verification depth
        @param  str   ca_path       SSL server certification authority(ies) bundle path
        @param  str   cert_path     SSL client authentication certificate path/URI
        @param  str   key_path      SSL client authentication private key path/URI
        @throw  ValueError          Invalid argument value
        @throw  RuntimeError        Invalid argument (changing internal state)
        """

        # Validation
        # ... context
        if not context:
            raise ValueError("Missing SSL context (context)")
        if self.ssl_context is not None and self.ssl_context != context:
            raise RuntimeError(
                f"SSL context already initialized with different 'context' setting ({self.ssl_context})"
            )
        # ... ciphers
        if not ciphers:
            raise ValueError("Missing SSL context ciphers (ciphers)")
        if (
            self.ssl_context_ciphers is not None
            and self.ssl_context_ciphers != ciphers
        ):
            raise RuntimeError(
                f"SSL context already initialized with different 'ciphers' setting ({self.ssl_context_ciphers})"
            )
        # ... verify
        if not verify:
            verify = False
        if (
            self.ssl_context_verify is not None
            and self.ssl_context_verify != verify
        ):
            raise RuntimeError(
                f"SSL context already initialized with different 'verify' setting ({self.ssl_context_verify})"
            )
        # ... verify depth
        if not verify_depth:
            verify_depth = 1
        if (
            self.ssl_context_verify_depth is not None
            and self.ssl_context_verify_depth != verify_depth
        ):
            raise RuntimeError(
                f"SSL context already initialized with different 'verify_depth' setting ({self.ssl_context_verify_depth})"
            )
        # ... certification authority
        if not ca_path:
            ca_path = ""
        if (
            self.ssl_context_ca_path is not None
            and self.ssl_context_ca_path != ca_path
        ):
            _ca_path = (
                self.ssl_context_ca_path
                if self.ssl_context_ca_path
                else "<system>"
            )
            raise RuntimeError(
                f"SSL context already initialized with different 'ca_path' setting ({_ca_path})"
            )
        # ... certificate
        if not cert_path:
            cert_path = ""
        if (
            self.ssl_context_cert_path is not None
            and self.ssl_context_cert_path != cert_path
        ):
            _cert_path = (
                self.ssl_context_cert_path
                if self.ssl_context_cert_path
                else "<none>"
            )
            raise RuntimeError(
                f"SSL context already initialized with different 'cert_path' setting ({_cert_path})"
            )
        # ... private key
        if not key_path:
            key_path = ""
        if (
            self.ssl_context_key_path is not None
            and self.ssl_context_key_path != key_path
        ):
            _key_path = (
                self.ssl_context_key_path
                if self.ssl_context_key_path
                else "<none>"
            )
            raise RuntimeError(
                f"SSL context already initialized with different 'key_path' setting ({_key_path})"
            )

        # Preload the SSL context
        if self._ssl_context is not None:
            return
        ssl_context = M2Crypto.SSL.Context(context)
        ssl_context.set_cipher_list(ciphers)

        # ... server verification (certification authority)
        if verify:
            ssl_context.set_verify(
                mode=M2Crypto.SSL.verify_peer
                | M2Crypto.SSL.verify_fail_if_no_peer_cert,
                depth=verify_depth,
            )
            if len(ca_path):
                if os.path.isfile(ca_path):
                    ssl_context.load_verify_locations(cafile=ca_path)
                else:
                    ssl_context.load_verify_locations(capath=ca_path)
            else:
                ssl_context.set_default_verify_paths()
        else:
            ssl_context.set_allow_unknown_ca(True)

        # ... client authentication (certificate and private key)
        if len(cert_path) or len(key_path):
            if not len(cert_path):
                raise ValueError(
                    "Missing certificate path (along private key path)"
                )
            if not len(key_path):
                raise ValueError(
                    "Missing private key path (along certificate path)"
                )

            # Load SSL engine
            # ... for RFC7512 PKCS#11 URIs
            if cert_path[0:7] == "pkcs11:" or key_path[0:7] == "pkcs11:":
                self.init_ssl_engine(id="pkcs11")

            # Load certificate and private key
            # ... via SSL engine (if loaded)
            if self._ssl_engine is not None:
                cert = self._ssl_engine.load_certificate(cert_path)
                key = self._ssl_engine.load_private_key(key_path)
                M2Crypto.m2.ssl_ctx_use_x509(ssl_context.ctx, cert.x509)
                M2Crypto.m2.ssl_ctx_use_pkey_privkey(ssl_context.ctx, key.pkey)
            # ... from file
            else:
                ssl_context.load_cert(certfile=cert_path, keyfile=key_path)

        # Save SSL context settings
        self._ssl_context = ssl_context
        self.ssl_context = context
        self.ssl_context_ciphers = ciphers
        self.ssl_context_verify = verify
        self.ssl_context_verify_depth = verify_depth
        self.ssl_context_ca_path = ca_path
        self.ssl_context_cert_path = cert_path
        self.ssl_context_key_path = key_path
