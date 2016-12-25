#!/bin/bash

## build binutils
wget http://ftp.gnu.org/gnu/binutils/binutils-2.27.tar.bz2
tar xvfj binutils-xxxx
./configure --target=rx-elf --prefix=/usr/local/rx-elf
make
sudo make install

## GMP
wget https://gmplib.org/download/gmp/gmp-6.1.1.tar.bz2
tar xvfj gmp-xxxx
.configure
make
sudo make install
make check

## MPFR
get http://www.mpfr.org/mpfr-current/mpfr-3.1.5.tar.bz2
tar xvfz mpfr-xxxx
./configure --with-gmp=/usr/local
make
sudo make install

## MPC
get ftp://ftp.gnu.org/gnu/mpc/mpc-1.0.3.tar.gz
tar mpc-xxxx
./configure --with-gmp=/usr/local
make
sudo make install

## newlib for stdlib
wget 
tar xvfj newlib-xxxx

## make dir for gcc build
get http://ftp.tsukuba.wide.ad.jp/software/gcc/releases/gcc-5.4.0/gcc-5.4.0.tar.bz2
tar xvfj gcc-xxxx
cd gcc-5.4.0
ln -s ../newlib-xxx/newlib .
ln -s ../newlib-xxx/libgloss .
mkdir build_gcc-a.b.c
cd build build_gcc-a.b.c
../gcc-5.4.0/configure --prefix=/usr/local/rx-elf --target=rx-elf --with-gmp=/usr/local --disable-libssp --enable-languages=c --with-newlib
make
sudo make install
