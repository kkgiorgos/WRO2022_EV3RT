bash --login ./BuildScripts/make.sh
$appName = 'RUN2022'
$driveLetter = 'D'
Copy-Item .\$appName $driveLetter':/ev3rt/apps/'
