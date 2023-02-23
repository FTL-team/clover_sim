pushd src/lib
cargo test
popd
pushd src/web
yarn
rm dist
yarn build --mode production
popd
cargo build -r --target=x86_64-unknown-linux-musl