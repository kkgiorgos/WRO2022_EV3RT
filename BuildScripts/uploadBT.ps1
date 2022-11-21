bash --login ./BuildScripts/make.sh
$appName = 'runWRO'
$ip = '10.0.10.1'
curl.exe -f --noproxy "*" -H "Content-Type: ev3rt/app" -H "Content-Disposition: inline; filename=\`"$appName\`"" --data-binary "@$appName" --url http://$ip/upload > uploadOutput.txt
Remove-Item uploadOutput.txt
