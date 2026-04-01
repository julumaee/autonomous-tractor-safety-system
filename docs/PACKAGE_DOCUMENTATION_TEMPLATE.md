# Package Documentation Template

Use this template for every perception-system package.
Keep the package-level part short, and place the main documentation under node-specific subsections.

---

# [Package Name]

## 1. Package purpose
- **Package:**
- **Primary path:**
- **Purpose:**

Write a short explanation of the package purpose.
Keep this section brief. It should only explain what the package is for and where it fits in the overall system.

---

## 2. Nodes

List all nodes in the package first.

| Node | Source file | Short purpose |
|---|---|---|
| `node_1` | `node_1.py` | Brief one-line description |
| `node_2` | `node_2.py` | Brief one-line description |

Then document each node with the same subsection structure.

### 2.1 `node_1`

#### 2.1.1 Overview
- **Node name:**
- **Executable:**
- **Source file:**
- **Purpose:**

Briefly explain what this node does and why it exists.

#### 2.1.2 Topics and interfaces

##### Subscribed topics
| Topic | Message type | Callback / handler | Purpose | Notes |
|---|---|---|---|---|
| `/example_input` | `example_msgs/ExampleMsg` | `example_callback()` | Explain the input | Rate, frame, assumptions |

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/example_output` | `example_msgs/ExampleMsg` | `publish_example()` | Explain the output | Timing, downstream user, frame |

##### Other interfaces (optional)
- **Services:**
- **Actions:**
- **TF frames used or produced:**
- **Timers:**

#### 2.1.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---|---|---|---|
| `example_param` | `float` | `0.5` | `m` | `example_function()` | Explain exactly what the parameter controls |

#### 2.1.4 Functions
Document every function or method in this node.

##### `function_name()`
- **Purpose:**
- **Called by:**
- **Inputs:**
- **Returns:**
- **Main steps:**
  1.
  2.
  3.
- **Parameters used:**
- **Notes:**

##### `another_function()`
- **Purpose:**
- **Called by:**
- **Inputs:**
- **Returns:**
- **Main steps:**
  1.
  2.
  3.
- **Parameters used:**
- **Notes:**

> Repeat until every function in the node has been documented.

#### 2.1.5 Processing flow
Describe this node's operation from input to output as a short numbered sequence.

Example:
1. The node receives data from subscribed topics.
2. Incoming data is validated, transformed, or buffered.
3. Core processing is executed in callbacks and/or timer loops.
4. Internal state is updated.
5. Results are published.

#### 2.1.6 Notes / failure cases
List practical notes, assumptions, limitations, and failure cases for this node.

- Notes:
  -
  -
- Failure cases:
  -
  -

### 2.2 `node_2`

#### 2.2.1 Overview
- **Node name:**
- **Executable:**
- **Source file:**
- **Purpose:**

Briefly explain what this node does and why it exists.

#### 2.2.2 Topics and interfaces

##### Subscribed topics
| Topic | Message type | Callback / handler | Purpose | Notes |
|---|---|---|---|---|
| `/example_input` | `example_msgs/ExampleMsg` | `example_callback()` | Explain the input | Rate, frame, assumptions |

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/example_output` | `example_msgs/ExampleMsg` | `publish_example()` | Explain the output | Timing, downstream user, frame |

##### Other interfaces (optional)
- **Services:**
- **Actions:**
- **TF frames used or produced:**
- **Timers:**

#### 2.2.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---|---|---|---|
| `example_param` | `float` | `0.5` | `m` | `example_function()` | Explain exactly what the parameter controls |

#### 2.2.4 Functions
Document every function or method in this node.

##### `function_name()`
- **Purpose:**
- **Called by:**
- **Inputs:**
- **Returns:**
- **Main steps:**
  1.
  2.
  3.
- **Parameters used:**
- **Notes:**

> Repeat until every function in the node has been documented.

#### 2.2.5 Processing flow
Describe this node's operation from input to output as a short numbered sequence.

#### 2.2.6 Notes / failure cases
List practical notes, assumptions, limitations, and failure cases for this node.

- Notes:
  -
  -
- Failure cases:
  -
  -

> Add sections `2.3`, `2.4`, etc. for the remaining nodes in the package.

---

## 3. Tests in `/test`

Explain the tests that belong to this package.

### Test overview
- What kinds of tests exist?
- What they verify?
- How they are organized under `/test`?

### Test files
| Test file | Target | Type | Purpose | Notes |
|---|---|---|---|---|
| `test_example.py` | `node_1` | unit / integration / launch | Explain what it checks | Optional comments |

### Running the tests
- **Command(s):**
- **Prerequisites:**
- **Expected environment:**

### Notes on test coverage
- Which parts of the package are covered?
- Which parts are not yet covered?
- Any known limitations of the test setup?

---

# Short authoring checklist

Before considering a package document complete, check that it includes:

- [ ] short package-purpose section
- [ ] full list of package nodes
- [ ] for each node: overview
- [ ] for each node: topics and interfaces
- [ ] for each node: parameters
- [ ] for each node: functions / methods
- [ ] for each node: processing flow
- [ ] for each node: notes / failure cases
- [ ] explanation of tests under `/test`
