Robot Commands:

- ClimbLockCommands.java
  - getClimbLockCommand
    - command factory which builds and returns the cage lockign sequence commands
    - sequence:
      - pre-lock LED sequence (TBD)
      - close locks and stall lock motor to hold
      - post-lock LED sequence (TBD)
