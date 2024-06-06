## 1 Basic Database Structure 

The original code is included in `database.py`

### 1.1 Chemical Substance:
- The `Chemical` class is used to describe the properties of chemical substances.

```python
class Chemical:
    """
    Initialize the attributes of the chemical substance
    - name: Name of the chemical substance
    - color: Color (optional)
    - state: State (e.g., gas, liquid, solid, etc.)
    - enthalpy: Enthalpy value (unit: J/mol)
    """
    
    def __init__(self, name, color=None, enthalpy=None, state=None):
        self.name = name
        self.color = color
        self.state = state
        self.enthalpy = enthalpy
```

- The `ChemicalDatabase` class is used to manage a collection of chemical substances and provide lookup functionality.
```python
class ReactionDatabase:
    """
    Find the corresponding chemical object by name <br> `return`: `Chemical` object or `None`
    """
class ChemicalDatabase:
    def __init__(self, chemicals):
        self.chemicals = chemicals

    def find(self, chemical_name):
        chemical = self.chemicals.get(chemical_name)
        if chemical:
            return chemical
        return None
```


### 1.2 Chemical Reactions:
- The `Reaction` class is used to represent ionic reactions.

```python
class Reaction:
    """
    Initialize the attributes of the ionic reaction
    - `reactants`: List of reactants
    - `products`: List of products
    - `mole_ratio`: Dictionary of mole ratios (example: {'reactant_A': 2, 'reactant_B': 1, 'product_C': 3})
    - `enthalpy`: Enthalpy change of the reaction (unit: J)
    """
    def __init__(self, reactants, products, mole_ratio):
        self.reactants = reactants
        self.products = products
        self.mole_ratio = mole_ratio
        self.enthalpy = 0
        for reactant in reactants:
            self.enthalpy += reactant.enthalpy * self.mole_ratio[reactant]
        for product in products:
            self.enthalpy -= product.enthalpy * self.mole_ratio[product]
```

The `ReactionDatabase` class is used to manage a collection of ionic reactions and provide functionality to lookup reactions based on reactants.
```python
class ReactionDatabase:
    """
    Find the corresponding ionic reaction object by reactants <br> `return`: `Reaction` object or `None`
    """
    def __init__(self, reactions):
        self.reactions = reactions

    def get_reaction(self, reactants):
        for reaction in self.reactions:
            if set(reactants).issuperset(set(reaction.reactants)):
                return reaction
        return None
```



## 2 Simulator 

The original code is included in `simulator.py`

### 2.1 Simulator Class: ChemicalReactionSimulator

The `ChemicalReactionSimulator` class is used to simulate ionic reaction processes and provides functionality to calculate the characteristics of the mixed solution.

```python
class ChemicalReactionSimulator:
    def __init__(self, re_db, ch_db, verbose=True):
        self.re_db = re_db
        self.ch_db = ch_db
        self.enthalpy = 0
        self.volume = 0
        self.tem = 25
        self.state = set()
        self.flag = 1
        self.verbose = verbose
    
    # simulate the ionic reaction process. 
    def simulate_reaction(self, input, input_sol_vol, temp=25):
    
    # Calculate rgba color for single reagent
    def color_dilution(self, org_name, org_conc):
    
    # Calculate mixed color
    def mix_solutions(self, products):
    
    # Calculate pH value
    def ph_calculation(self, product):
    
    # Output all characterization results
    def get_information(self, result):

    # After each simulation, reset the attributes contained in the simulator
    def reset(self):
```


### Method Details

#### `simulate_reaction(input, input_sol_vol)`
-   **Input**
    - `input`: A dictionary containing the moles of reactants.
    - `input_sol_vol`: A list containing the volume of solvents.

-   **Implementation Logic:**
    - Loop the simulation process: This method repeatedly executes the following steps until equilibrium is reached or specific conditions trigger an exit from the loop.
    - Equilibrium Judgement: Use `charge_balance()` to determine if the charges of the reactants are balanced. If unbalanced, return "Unbalance Charge".
    - Reaction Type Determination: Based on the type of reactants, obtain the corresponding reaction information from the ionic reaction database.
    - Initialize Reactants and Products: Initialize the substances involved in the reaction and their mole numbers according to the mole ratio, and calculate the mole numbers in a 1 mol equation.
    - Update Moles of Reactants and Products: Update the moles of the substances involved in the reaction and the products based on the mole ratio of the reaction.
    - Enthalpy and Temperature Calculation: Calculate the enthalpy change and temperature change based on the enthalpy change and mole numbers of the reaction, and update the temperature using `temperature()`.
    - Simulation End Condition: If the simulation result is the same as the previous result, end the simulation; if the temperature is not within the range of 0~100 degrees, calculate it as 0 or 100.

-   **Return Value:**
    - A dictionary of reaction products after simulation or specific information (e.g., "Unbalance Charge").

#### `mix_solutions(products)`


-   **Input**
    - `products`: A dictionary containing the moles of products.

-   **Implementation Logic:**
    - Mix Solutions: Based on the given products, call the `color_dilution()` method for each substance to obtain the color at the current concentration.
    - Color Calculation: Use the `mix_color()` method to calculate the final color based on the mixed solution colors.

-   **Return Value:**
    - An rgba list containing the color of the mixed solution.


#### `color_dilution(org_name, org_conc)`

-   **Input**
    - `org_name`: Name of a substance
    - `org_conc`: Concentration of a substance

-   **Implementation Logic:**
    - Distinguish between the opacity of three states of matter
    - Use the formula `a = 1 - 10 ** (-K_const * add_conc)` to calculate the opacity of liquids at different concentrations
    - `K_const` can change the overall opacity

-   **Return Value:**
    - rgba list

#### `get_information(products)`


-   **Input**
    - `products`: A dictionary containing the moles of products.

-   **Implementation Logic:**
    - Output `enthalpy`,`color`,`temperature`,`ph`,`volume`,`state` in turn

-   **Return Value:**
    - A dictionary containing the characterization of the products


#### `ph_calculation(product)` 

- **Input**
    - `products`: A dictionary containing the moles of products.

- **Implementation Logic:**
  - Determine based on products: Determine whether the product is acidic or alkaline.
  - Ionization constant calculation: Calculate the ionization constant of water based on the current temperature `temperature` by calling `ionization_constant(tem)`.
  - pH calculation: Calculate the pH value based on the acidity or alkalinity of the substance.
  - Return pH value: Return the calculated pH value.

 -   **Return Value:**
    - The calculated pH value.

#### `reset()`

- **Return Value:**
    - None


### 2.2 Simulator Class: OrganicReactionSimulator

The `OrganicReactionSimulator` class is used to simulate organic reaction processes and can predict product generation and query product properties.
```python
class OrganicReactionSimulator:
  
    # Initialization method. Set the RXN4Chemistry API key and initialize the required variables.
    def __init__(self, tem=None):
        api_key = ...
        self.rxn4chemistry_wrapper = RXN4ChemistryWrapper(api_key=api_key)
        self.rxn4chemistry_wrapper.create_project('organic')
        self.tem = tem
        self.formula = None
        self.reactant = None
        self.product = None
        self.reactant_mass = 0
        self.product_mass = 0

    # Used to simulate organic reactions.
    def simulate_reaction(self, input):
  
    # Method to obtain product information.
    def get_information(self):
```

### Method Details


#### `simulate_reaction(input)`

- **Input**: 
   - A dictionary containing SMILES symbols as keys and masses as values.

- **Implementation Logic:**
  - Calculate the total mass of reactants `reactant_mass`.
  - Initiate a request for chemical reaction prediction.
  - Parse the response and extract product information `self.formula`.
  - Use the predicted yield `chanlv` to calculate the moles of products `self.product_mass`.

- **Return Value:**
  - A dictionary of products and their masses.

#### `get_info()`

- **Input**: 
   - None.
  
- **Implementation Logic:**
  - Call `get_info_organic()` toQuery the SMILES of the product on the ChemSpider website.
  - Crawl the substance information of Properties - Predicted-ACD/Labs and extract it into a dictionary.
  - Supplement with name, molecular formula, SMILES, cas_number, melting point.

- **Return Value:**
   - Dictionary containing product state and characterization.
