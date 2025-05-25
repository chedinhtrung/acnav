## Mother ship shared library
This library contains code to handle common functionalities on all mothership devices, such as: 

- Common data structures
- Message queues
- Node
- Datalink
- Math
- State Estimation utilities
- GPS navigation utilities
- Device drivers
- Bundled dependencies

## Platform specific builds
Since Arduino platform and Linux servers use .a files and Windows MSVC use .lib, there has to be 2 build directories. 
MinGW cannot be used on Windows due to WebEngineView not supported on MinGW.

