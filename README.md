## To do:
- Add a command to enable/disable data streaming
    - In stream mode, the controller will regularly send a data/status packet
        - Controller status
        - Data (target/measured speed, encoder counts, control signal)
    - The streamed data packet will have its own special command code
    - Need to determine the period for this, maybe every 5-10 ms
    - Regular command/response proceeds in parallel
