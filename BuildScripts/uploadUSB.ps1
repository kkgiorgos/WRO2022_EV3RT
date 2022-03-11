bash --login ./BuildScripts/make.sh
$appName = 'runWRO'
$driveLetter = 'F'
Copy-Item .\$appName $driveLetter':/ev3rt/apps/'
