<?php
$file = 'people.txt';
// The new person to add to the file
$person = "John Smith\n";
// Write the contents to the file, 
// using the FILE_APPEND flag to append the content to the end of the file
// and the LOCK_EX flag to prevent anyone else writing to the file at the same time
file_put_contents($file, $person, FILE_APPEND | LOCK_EX);



file_put_contents("user_code.py", $_POST["code"], LOCK_EX);
system("cat code_template_part_1.py user_code.py code_template_part_2.py > combined_code.py");
system("python combined_code.py");
?>
