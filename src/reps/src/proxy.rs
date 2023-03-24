use axum::{
    extract::{ws::Message, WebSocketUpgrade},
    headers::ContentType,
    response::IntoResponse,
    routing::{any, get, MethodRouter},
    TypedHeader,
};
use http::{Request, StatusCode, Uri, header::{HOST, ORIGIN}, HeaderValue};
use hyper::{upgrade::OnUpgrade, Body, Client};
use tokio::{
    io::{copy_bidirectional, AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    select,
};

const NO_CONNECTION_PAGE: &str = "
<style>
h1{font-size:3em;margin-bottom:0}main{border:4px solid red;background:#f99;
position:fixed;height:120px;margin-top:-60px;width:420px;margin-left:-210px;
left:50%;top:50%;text-align:center;font-family:\"Courier New\",monospace;
padding:15px;box-sizing:border-box}main *{margin:0}
</style>
<script>setTimeout(() => location.reload(), 1000)</script>

<main>
<h1>Disconnected</h1>
<h2>New attempt in few seconds</h2>
</main>
";

pub fn http_proxy_service(target: String) -> MethodRouter<(), Body> {
    let client = Client::new();
    any(|mut req: Request<Body>| async move {
        let path = req.uri().path();
        let path_query = req
            .uri()
            .path_and_query()
            .map(|v| v.as_str())
            .unwrap_or(path);

        let uri = target + path_query;
        let uri = Uri::try_from(uri).unwrap();

        let host_port = format!("{}:{}", uri.host().unwrap(), uri.port().unwrap());
        req.headers_mut().insert(HOST, HeaderValue::from_str(&host_port).unwrap());
        req.headers_mut().insert(ORIGIN, HeaderValue::from_str(&format!("http://{}", host_port)).unwrap());
        *req.uri_mut() = uri;


        let request_upgraded = req.extensions_mut().remove::<OnUpgrade>();

        let res = client.request(req).await;
        let Ok(mut res) = res else {
          return (StatusCode::BAD_GATEWAY, TypedHeader(ContentType::html()), String::from(NO_CONNECTION_PAGE)).into_response();
        };

        if res.status() == StatusCode::SWITCHING_PROTOCOLS {
            if let Some(request_upgraded) = request_upgraded {
                let mut response_upgraded = res
                    .extensions_mut()
                    .remove::<OnUpgrade>()
                    .expect("response does not have an upgrade extension")
                    .await
                    .unwrap();

                tokio::spawn(async move {
                    let mut request_upgraded =
                        request_upgraded.await.expect("failed to upgrade request");
                    #[allow(unused_must_use)]{
                        copy_bidirectional(&mut response_upgraded, &mut request_upgraded)
                            .await;
                    }
                });

                res.into_response()
            } else {
                (
                    StatusCode::BAD_REQUEST,
                    String::from("Cannot switch to a new protocol"),
                )
                    .into_response()
            }
        } else {
            res.into_response()
        }
    })
}

pub fn tcp_proxy_route(target: String) -> MethodRouter<(), Body> {
    get(|ws: WebSocketUpgrade| async move {
        ws.on_upgrade(|mut socket| async move {
        let mut stream = TcpStream::connect(target).await.unwrap();
        let mut readbuf: [u8; 4096] = [0; 4096];
        loop {
            select! {
                len = stream.read(&mut readbuf) => {
                    dbg!(&len);
                    if let Ok(len) = len {
                        socket.send(axum::extract::ws::Message::Binary(readbuf[..len].to_vec())).await.unwrap();
                    } else {
                        break;
                    }
                }
                msg = socket.recv() => {
                    dbg!(&msg);
                    let Some(msg) = msg else {
                        break;
                    };
                    let Ok(msg) = msg else {
                        break;
                    };
                    if let Message::Close(_) = msg {
                        break;
                    }
                    stream.write(&msg.into_data()).await.unwrap();
                    // msg.
                }
            }
        }
    })
    })
}
