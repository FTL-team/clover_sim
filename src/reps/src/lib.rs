mod proxy;

use std::net::{Ipv6Addr, SocketAddr};
use std::sync::Arc;

use axum::extract::ws::Message::Text;
use axum::extract::{Path, WebSocketUpgrade};
use axum::headers::ContentType;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router, TypedHeader};
use cloversim_lib::launch_event::LaunchEvent;
use cloversim_lib::rpc::NodeRpc;
use http::{header, StatusCode};
use include_dir::{include_dir, Dir};
use tokio::select;
use tower_http::set_header::SetResponseHeaderLayer;

static WEBUI_ASSETS: Dir = include_dir!("$CARGO_MANIFEST_DIR/../../src/web/dist");

pub async fn get_api_routes(rpc: Arc<dyn NodeRpc>) -> Router {
    let app = Router::new();

    let route_rpc = rpc.clone();
    let app = app.route(
        "/status",
        get(move || async move { Json(route_rpc.status().await) }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/list_workspaces",
        get(|| async move { Json(route_rpc.list_workspaces().await) }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/list_tasks",
        get(|| async move { Json(route_rpc.list_tasks().await) }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/create_workspace",
        post(|name: Json<String>| async move { Json(route_rpc.create_workspace(name.0).await) }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/remove_workspace",
        post(|name: Json<String>| async move { Json(route_rpc.remove_workspace(name.0).await) }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/duplicate_workspace",
        post(|inp: Json<(String, String)>| async move {
            Json(route_rpc.duplicate_workspace(inp.0 .0, inp.0 .1).await)
        }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/clean_workspace",
        post(|name: Json<String>| async move { Json(route_rpc.clean_workspace(name.0).await) }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/launch",
        post(
            |Json((ws_name, task_path)): Json<(String, String)>| async move {
                Json(route_rpc.launch(ws_name, task_path).await)
            },
        ),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/launch_channel",
        get(|ws: WebSocketUpgrade| async move {
            let rpc = route_rpc.clone();
            ws.on_upgrade(|mut socket| async move {
                let chan_tx = rpc.create_launch_channel().await.unwrap();
                let mut chan_rx = chan_tx.new_receiver();

                loop {
                    select! {
                         msg = socket.recv() => {
                            let Some(msg) = msg else {
                                break;
                            };
                            let Ok(msg) = msg else {
                                break;
                            };
                            let Text(msg) = msg else {
                                continue;
                            };
                            let Ok(msg) = serde_json::from_str::<LaunchEvent>(&msg) else {
                                continue;
                            };
                            let chan_rx_inactive = chan_rx.deactivate();
                            #[allow(unused_must_use)] { 
                                chan_tx.try_broadcast(msg);
                            }
                            chan_rx = chan_rx_inactive.activate();
                        }
                        msg = chan_rx.recv() => {
                            let Ok(msg) = msg else {
                                break;
                            };
                            let Ok(msg) = serde_json::to_string(&msg) else {
                                continue;
                            };
                            if socket.send(Text(msg)).await.is_err() {
                                break;
                            }
                        }
                    };
                }

                #[allow(unused_must_use)] {
                    socket.close().await;
                }
            })
        }),
    );

    let route_rpc = rpc.clone();
    let app = app.route(
        "/task/*file",
        get(|Path(path): Path<String>| async move {
            let mime_type = mime_guess::from_path(&path);

            let contents = route_rpc.read_task_file(path).await;
            match contents {
                Ok(contents) => {
                    let mime_type = mime_type.first_or(mime_guess::mime::APPLICATION_OCTET_STREAM);
                    (TypedHeader(ContentType::from(mime_type)), contents).into_response()
                }
                Err(err) => (StatusCode::BAD_REQUEST, Json(err)).into_response(),
            }
        }),
    );

    app.layer(SetResponseHeaderLayer::if_not_present(
        header::CONNECTION,
        header::HeaderValue::from_static("Keep-Alive"),
    ))
}

async fn serve_webui(Path(asset): Path<String>) -> Result<Response, StatusCode> {
    match WEBUI_ASSETS.get_file(&asset) {
        None => Err(StatusCode::NOT_FOUND),
        Some(f) => {
            let mime_type = mime_guess::from_path(&asset);
            let mime_type = mime_type.first_or(mime_guess::mime::APPLICATION_OCTET_STREAM);
            let response = (TypedHeader(ContentType::from(mime_type)), f.contents());
            Ok(response.into_response())
        }
    }
}

pub async fn run(rpc: Arc<dyn NodeRpc>) {
    let app = Router::new()
        .nest("/cloversim_api", get_api_routes(rpc).await)
        .nest_service(
            "/gzweb/",
            proxy::http_proxy_service(String::from("http://192.168.77.2:7777")),
        )
        .nest_service(
            "/webterm/",
            proxy::http_proxy_service(String::from("http://192.168.77.10:7776")),
        )
        .nest_service(
            "/ide/",
            proxy::http_proxy_service(String::from("http://192.168.77.10:7778")),
        )
        .nest_service(
            "/ros",
            proxy::http_proxy_service(String::from("http://192.168.77.10:9090")),
        )
        .route(
            "/ssh",
            proxy::tcp_proxy_route(String::from("192.168.77.10:22")),
        )
        .route("/", get(|| serve_webui(Path(String::from("index.html")))))
        .route("/*file", get(serve_webui));
    let addr = SocketAddr::from((Ipv6Addr::LOCALHOST, 7777));

    let server = axum::Server::bind(&addr)
        .serve(app.into_make_service());
    println!("Cloversim is listening on http://localhost:7777");
    let res: Result<(), hyper::Error> = server.await;
    res.unwrap();
}
