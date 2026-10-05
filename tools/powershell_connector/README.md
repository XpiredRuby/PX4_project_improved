# PowerShell connector timeout recovery

The original connector used `subprocess.run(capture_output=True)` on Windows.
After a timeout, Python kills the direct PowerShell process and drains its output
pipes. A descendant holding those pipes can keep that drain blocked beyond the
MCP request deadline. Input was also inherited from the MCP server.

`px4_connector_server.py` preserves the five existing MCP tool names and their
arguments. It closes command input, captures output in temporary files, and
waits for the direct process with a deadline. On timeout, Windows tree cleanup
has a five-second deadline and parent cleanup has a two-second deadline. The
command timeout is capped at 90 seconds to leave room before the observed
gateway deadline. Cleanup failures remain visible in the returned error.

Back up the installed `server.py`, copy this replacement over it, and restart
the PX4 tunnel process tree. The MCP environment and tunnel profile stay the
same. No credentials are part of this source.

Launch longer research jobs independently, with input closed and output saved
to files, then poll a status or exit file. For example, from Windows PowerShell:

```powershell
cmd.exe /d /c 'start "" /b C:\Windows\System32\wsl.exe -d Ubuntu-22.04 -- python3 -u /mnt/f/PX4/job.py < NUL > F:\PX4\job.out 2> F:\PX4\job.err'
```

The job must write its own exit/status file and perform simulation cleanup.
Only run simulation fault injection against a confirmed local SITL instance.

Run the connector regressions with:

```bash
python -m unittest discover -s tools/powershell_connector -p test_server.py -v
```

Five actual-process checks pass in the Linux preparation environment: closed
input, preserved exit status and stderr, inherited child output without a
pipe-drain hang, retained partial output on timeout, and bounded output tails.
The launch-failure path is also checked. Windows `taskkill` and a live MCP/WSL
session still require validation after installation. This connector repair does
not change flight control or establish the research project's readiness score.
