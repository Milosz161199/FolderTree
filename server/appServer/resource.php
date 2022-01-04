<?php
/**
 ******************************************************************************
 * @file    Data Grabber Example resource.php
 * @author  Milosz Plutowski
 * @version V1.0
 * @date    11.05.2021r.
 * @brief   Simple read data from ultrasonic sensor (server control data)
 ******************************************************************************
 */
error_reporting(E_ALL);

header("Content-Type: application/json");

function get_measurement($id) {
	switch ($id) {
		case 0:
			$res = shell_exec('sudo python3 /home/pi/rasptank_control/server/Measurements/ultrasonic.py');
			return $res;
		case 1:
			$res = shell_exec('sudo python3 /home/pi/rasptank_control/server/Measurements/cpuTemperature.py');
			return $res;
		case 2:
			$res = shell_exec('sudo python3 /home/pi/rasptank_control/server/Measurements/ramMemory.py');
			return $res;
		default: 
			return null;
	}
}


$id = 'ALL';
if(isset($_GET['id'])) 
{
	$id = json_decode($_GET['id']);
}

$response = array();
if(is_array($id)) 
{
	foreach($id as $i)
	{
		array_push($response, get_measurement($i));
	}
}
else if(is_integer($id)) 
{
  array_push($response, get_measurement($id));
}
else if($id == 'ALL') {
  for($i = 0; $i < 3; $i++)
    array_push($response, get_measurement($i));
}


header('Content-Type: application/json');
$tmp_string = '[ ';
for($i = 0 ; $i < count($response); $i++)
{
	if ($i == count($response) - 1)
	{
		$res = str_replace("\n", "", $response[$i]);
		$tmp_string = $tmp_string.$res;
	}
	else
	{
		$res = str_replace("\n", "", $response[$i]);
		$tmp_string = $tmp_string.$res.', ';
	}
}
$tmp_string = $tmp_string.' ]';
echo $tmp_string;

?>