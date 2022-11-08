bash --login ./BuildScripts/make.sh
$appName = 'runWRO'
$driveLetter = 'D'
Copy-Item .\$appName $driveLetter':/ev3rt/apps/'
