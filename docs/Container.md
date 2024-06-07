## 1 Reaction Container Usage

The original code is included in `inorganic_example.py`,`organic_example.py`

### 1.1 Container Class

The container binds chemical information to simulated reagent bottles, facilitating clear demonstrations. It also aligns chemical reactions with operational actions, making the simulation intuitive. 
This integrated simulating method allows for accurate predictions and detailed representation calculations.

```python
class Container:
    """
    initialize the reagent bottle or tube. Use the org parameter to distinguish organic and inorganic reaction.
    Only having the same org parameter can two containers use update method with each other.
    """
    def __init__(self, solute=None, org=False, volume=0, temp=25, verbose=False):
        self.solute = solute
        self.volume = volume
        self.temp = temp
        self.org = org
        self.simulator = OrganicReactionSimulator() if self.org else ChemicalReactionSimulator()
    
    # mixing, sampling or other operation
    def update(self, new_container, new_volume=None):
    
    # information retrieval method
    def get_info(self, verbose=False):
```

### 1.2 Methods Details
#### `update(input, new_container, new_volume=None)`
-   **Input**
    - `new_container`: Another container class object representing bottle, tubes and other chemical containers.
    - `new_volume`: The volume to mixing, setting `None` means all the incoming reagent.

- **Implementation Logic:**
  - Extract solute from the new container and set new_volume if not provided.
  - Compute the new temperature as a weighted average of the temperatures of both containers.
  - Increment the current container's volume by the new container's volume.
  - Update the current container's solute concentrations by adding the proportional amounts from the new container.
  - Adjust the new container's solutes and volume, and simulate any reactions in the new container.
  - Simulate reactions in the current container and update its temperature accordingly.
  - Generate a list of solute concentrations over time based on the current solutes and potential reactions.
  - Return the list of concentration values over time.

-   **Return Value:**
    - A list of intermediate state of the component during the operation if chemical reaction taken place.

#### `get_info(self, verbose=False)`
-   **Implementation Logic:**
    - The interface in simulator is invoked to obtain the representational content by means of computation or information retrieval

-   **Return Value:**
    - A dictionary containing all the information about the representations about the current product.

## 2 Examples and Discussions

### 2.1 Inorganic Example

- The IONIC REACTION section in `inorganic_example.py` provides examples
  - Initialization: Set up the container with specific reagent or emtpy.
  - Operation: Use `.update` method to sampling certain amount of reagent using a empty container, or mixing two container.
  - Mid-State: The intermediate state can be obtained as the output of `.update` method
  - Characterization Calculation: use `.get_info` method for any container. The returned dictionary gives a comprehensive description for the representations of the container.

### 2.2 Inorganic Part Debug
- Incorrect simulation components, please consider in order:
  - Missing reactants in `database.py - chemicals`
  - Missing reaction equations in `database.py - reactions`
  - Adjust the reaction order in `database.py - reactions`

- Incorrect enthalpy change calculation, please consider in order:
  - Missing or incorrect enthalpy change information for reactants in `database.py - chemicals`

- Incorrect temperature calculation, please consider in order:
  - Incorrect enthalpy change calculation
  - Temperature can only be expressed in the range [0,100]; increase the amount of solvent `input_solvent_vol` to reduce temperature change

- Incorrect pH calculation, please consider in order:
  - Incorrect temperature calculation, leading to incorrect `ionization_constant` calculation

- Incorrect color calculation, please consider in order:
  - Limitation: only considers color mixing, not the formation of complexes and chemical equilibria
  - Color information of substances in the database `database.py - chemicals`
  - Color formation of substances at a certain concentration: 2.2.1 `color_dilution` method
  - Color mixing function: `util.py - mix_color`
  - Global transparency adjustment: 2.2.1 `color_dilution` method, `K_const` can change overall transparency


### 2.3 Organic Example
- The ORGANIC REACTION section in `organic_example.py` provides examples
  - Initialization: Set up the container with specific reagent or emtpy, REMEMBER to set the parameter `org=True` for every container to use.
  - Operation: Use `.update` method to sampling certain amount of reagent using a empty container, or mixing two container.
  - Mid-State: The intermediate state can be obtained as the output of `.update` method
  - Characterization Calculation: use `.get_info` method for any container. The returned dictionary gives a comprehensive description for the representations of the container.

### 2.4 Organic Part Limitation

- States do not consider gases
- Product mass is calculated as crude yield
- For more information on substances, use cas_number for queries
