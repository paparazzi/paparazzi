#SBUS Fakerator
###Using
```bash
python main.py -$OPTIONS
```

###Options
There are the following options.
<table>
	<tr>
    	<th>option</th>
        <th>description</th>
        <th>args</th>
   	</tr>
	<tr>
    	<td>-i</td>
        <td>Activates the SIM's Ivy Interface and send data on it.</td>
        <td></td>
  	</tr>
    <tr>
    	<td>-l</td>
        <td>Artifically introduce latency</td>
        <td>Amount of latency introduced.</td>
   	</tr>
    <tr>
    	<td>-p</td>
        <td>Port</td>
        <td>Serial port address that the RC of the plane is connected to.</td>
  	</tr>

</table>

If there is no -i or -p (the output options) an error is thrown that terminates the program.

