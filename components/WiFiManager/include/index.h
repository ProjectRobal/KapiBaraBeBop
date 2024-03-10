#pragma once

const char *index_html = R"""(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Zarejestruj sieci</title>
    
    <style>

            *
            {
                font-family: Arial, Helvetica, sans-serif;
            }

            h1
            {
                text-align: center;
            }

            form
            {
                display: flex;
                flex-direction: column;
                flex-wrap: wrap;
                width: 100%;
                height: fit-content;
                margin: auto;
                gap:10px;
            }

            main
            {
                display: flex;
                width: 100%;
                height: 100%;
                align-items: center;
            }

        @media screen and (min-width: 600px) {
            form
            {
                width: 600px;
            }
        }

        html,body
        {
            height: 100%;
        }

    </style>
</head>
<body>
    <main>

    <form method="post" action="/ssid">

        <h1>Set a new network</h1>
        <p>SSID:</p>
        <input type="text" maxlength="32" name="ssid">
        <p>Password:</p>
        <input type="password" maxlength="64" name="password">

        <input type="submit" value="Connect">
    </form>
    
    </main>
</body>
</html>
)""";
unsigned int index_html_len = 1395;
