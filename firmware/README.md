# Overview
The system decodes Manchester-encoded data from the power line and categorizes messages based on their content. Once a new message is available, it is stored in a FIFO buffer, and the interrupt pin is set HIGH. The I2C master can then request the stored data. The relevant messages occus at round about 13Hz.

# Data Handling & Communication
## Manchester Code Decoding
The system continuously decodes Manchester-encoded data from the track’s power line.
Messages are categorized into predefined patterns for easy processing.

## FIFO Buffer & Interrupts
New data is stored in a FIFO buffer. When new messages arrive, the interrupt pin is set HIGH to signal the presence of new data.
Via I2C request a message from the FIFO. The system responds with the latest available data. The interrupt pin is cleared once the FIFO is read.

# Data Message Structure
Each data chunk is 16 bits (2 bytes) long and consists of an identifier and a value:
Bits    Field   Description
15-8    Identifier  Defines the type of message (e.g., car status, system events)
7-0 Value   Stores specific data related to the identifier (e.g., speed, lap time)

## Message Types & Decoding

The identifier (high byte) determines the type of data, while the value (low byte) contains specific information.
### Car-Related Messages
| Identifier (Hex)  | Description |
|------:|-----|
| 0xFF0Z    |  Car with ID Z finishes the race |
| 0xFEXZ    |  Car with ID Z crosses the lap counter; if X = 1, it records a new fastest lap    |
| 0xFDXZ    |  Speed of car Z, where Z is a 4-bit car ID, and X is a 12-bit speed value |
| 0xFCXZ    |  Fuel level of car Z (X is a 3-bit integer fuel value)    |
| 0xFBXZ    |  Throttle position of car Z   |
| 0xFA0Z    |  Car Z jumped the start   |
### System Messages
| Identifier (Hex)  | Description |
|------:|-----|
| 0xAA0X    |  Number of start lights illuminated (X = 0 to 5) |
| 0xAB0X    |   Yellow flag (X = 1 yellow flag, X = 0 yellow flag withdrawn |
| 0x0001    |  Total system reset |
| 0x0002    |  Reset position and lap counter |

## Example Data Transactions
### Example 1: Car 3 Finishes the Race
Raw Message: 0xFF03
Decoded:

    Identifier: 0xFF → Car finishes race. 
    Car ID: 03 
    Description: Car 3 finished the race.

### Example 2: Start lights changed
Raw Message: 0xAA03
Decoded:
    
    Identifier: 0xAA → Start lights changed
    Value: 3
    3 red lights lit at the start light

# Notes & Considerations
The interrupt pin remains HIGH until the FIFO is read.

# Contribute & Support

Got ideas? Found a bug? Want to make it even cooler? 
Open an issue, submit a PR, or just say hi!

    Developer: Philipp Krebs 
    Contact: pmge.krebs@gmail.com 
    Repo: https://github.com/philippmichaelkrebs 

