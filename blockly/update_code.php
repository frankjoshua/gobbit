<?php
file_put_contents("user_code.py", $_POST["code"], LOCK_EX);
system("cat code_template.py user_code.py > combined_code.py");
?>