#!/bin/bash

compDir=YS-ev3rt-hrp2
appName=runWRO

rm ~/EV3RT/$compDir/sdk/workspace/app
rm -rf ~/EV3RT/$compDir/sdk/workspace/project

cp -r . ~/EV3RT/$compDir/sdk/workspace/project

path=`pwd`

cd ~/EV3RT/$compDir/sdk/workspace/project
mv -v ./Sources/* ./
mv -v ./Headers/* ./
mv -v ./ConfigFiles/* ./
cd ../
make app=project
cd $path

cp -r ~/EV3RT/$compDir/sdk/workspace/app .
mv app $appName

cp -rv $path/$appName  /mnt/d/ev3rt/apps

rm ~/EV3RT/$compDir/sdk/workspace/app
rm -rf ~/EV3RT/$compDir/sdk/workspace/project
