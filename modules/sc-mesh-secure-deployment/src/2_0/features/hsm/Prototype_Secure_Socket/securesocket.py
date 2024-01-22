
import socket
import ssl
import argparse

pkcs11 = None

def wrap_socket(sock, ssl_params={}):
    if any(ssl_params[k] is None for k in ("keyfile", "certfile", "pin")):
        print("[+] Using filebased TLS")
        context = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH,
                                             cafile=ssl_params.get("cafile", None))
        context.minimum_version = ssl.TLSVersion.TLSv1_3
        context.check_hostname = False

        cert_verify = ssl_params.get("cert_verify", "none")
        if cert_verify == "none":
            context.verify_mode = ssl.CERT_NONE
        elif cert_verify == "optional":
            context.verify_mode = ssl.CERT_OPTIONAL
        else:
            context.verify_mode = ssl.CERT_REQUIRED

        if all(k in ssl_params for k in ("keyfile", "certfile")):
            context.load_cert_chain(
                certfile=ssl_params["certfile"],
                keyfile=ssl_params["keyfile"],
            )
        return context.wrap_socket(sock)

    print("[+] Using HSM TLS")
    from M2Crypto import m2, SSL, Engine

    global pkcs11
    if pkcs11 is None:
        pkcs11 = Engine.load_dynamic_engine(
            b"pkcs11", ssl_params["engine_path"]
        )
        pkcs11.ctrl_cmd_string(
            "MODULE_PATH", ssl_params["module_path"]
        )
        pkcs11.ctrl_cmd_string("PIN", ssl_params["pin"])
        pkcs11.init()


    ctx = SSL.Context("tls")
    ctx.set_default_verify_paths()
    ctx.set_allow_unknown_ca(False)

    ciphers = ssl_params.get("ciphers", None)
    if ciphers is not None:
        ctx.set_cipher_list(ciphers)

    cafile = ssl_params.get("cafile", None)
    if cafile is not None:
        if ctx.load_verify_locations(cafile) != 1:
            raise Exception("Failed to load CA certs")

    cert_verify = ssl_params.get("cert_verify", "none")
    if cert_verify == "none":
        cert_verify = SSL.verify_none
    else:
        cert_verify = SSL.verify_peer

    ctx.set_verify(cert_verify, depth=9)

    key = pkcs11.load_private_key(ssl_params["keyfile"])
    cert = pkcs11.load_certificate(ssl_params["certfile"])

    m2.ssl_ctx_use_pkey_privkey(ctx.ctx, key.pkey)
    m2.ssl_ctx_use_x509(ctx.ctx, cert.x509)

    SSL.Connection.postConnectionCheck = None

    return SSL.Connection(ctx, sock=sock)


def main(pin, cert_path, key_path, ca_path, cert_verify, engine_path, module_path):
    ssl_params = {
        "pin" : pin,
        "keyfile" : key_path,
        "certfile" : cert_path,
        "cafile" : ca_path,
        "cert_verify" : cert_verify,
        "engine_path" : engine_path,
        "module_path" : module_path
    }

    sock = socket.socket()
    ssock = wrap_socket(sock, ssl_params)
    ssock.bind(('127.0.0.1', 4433))
    ssock.listen(5)

    while True:
        conn, ip = ssock.accept()

        print(f"[+] New connection from {':'.join(map(str, ip))}")

        conn.shutdown(socket.SHUT_RDWR)
        conn.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--pin', help='HSM PIN')
    parser.add_argument('--cert-path', help='Certificate file path')
    parser.add_argument('--key-path', help='Key file path')
    parser.add_argument('--ca-path', help='CA path')
    parser.add_argument('--cert-verify', help='Certificate verification (none, optional, required)')
    parser.add_argument('--engine-path', default='/usr/lib/x86_64-linux-gnu/engines-3/libpkcs11.so', help='HSM Engine')
    parser.add_argument('--module-path', default='/usr/lib/softhsm/libsofthsm2.so', help='HSM Module')

    args = parser.parse_args()

    try:
        main(args.pin, args.cert_path, args.key_path, args.ca_path, args.cert_verify, args.engine_path, args.module_path)
    except (Exception, KeyboardInterrupt):
        print("\n[!] Exiting")
