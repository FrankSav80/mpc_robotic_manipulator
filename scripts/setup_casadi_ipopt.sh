#!/bin/bash

set -e

echo "======================================"
echo " Installing IPOPT (solver dependency)"
echo "======================================"

sudo apt update
sudo apt install -y coinor-libipopt-dev


echo ""
echo "======================================"
echo " Installing CasADi (from source)"
echo "======================================"

CASADI_WS=~/casadi_ws

mkdir -p "$CASADI_WS"
cd "$CASADI_WS"

if [ ! -d "casadi" ]; then
    git clone https://github.com/casadi/casadi.git -b main
fi

cd casadi
mkdir -p build
cd build

cmake .. -DWITH_IPOPT=ON -DWITH_LAPACK=ON
make -j"$(nproc)"

sudo make install
sudo ldconfig


echo ""
echo "======================================"
echo " Verifying installation"
echo "======================================"

if ldconfig -p | grep -q casadi; then
    echo "CasADi found"
else
    echo "CasADi not found"
fi

if ldconfig -p | grep -q ipopt; then
    echo "IPOPT found"
else
    echo "IPOPT not found"
fi


echo ""
echo "Setup completed successfully."