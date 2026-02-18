<a id="readme-top"></a>

<!-- PROJECT BANNER -->
<p align="center">
  <img src="assets/banner_section_1.png" alt="Langostino — open autonomous drone" width="100%" />
</p>

<h1 align="center">Langostino — The Swarm Drone</h1>

<p align="center">
  <b>Open-source AI autopilot for the real world: understand it, build it, and fly it.</b><br/>
  An open-source autonomous flight reference platform, built with a <b>global</b> community.
</p>

<!-- BADGES -->
<p align="center">
  <a href="https://github.com/swarm-subnet/Langostino/stargazers">
    <img alt="GitHub Stars" src="https://img.shields.io/github/stars/swarm-subnet/Langostino?style=flat-square" />
  </a>
  <a href="LICENSE.md">
    <img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow?style=flat-square" />
  </a>
  <img alt="Version" src="https://img.shields.io/badge/Version-v1.0.0-black?style=flat-square" />
  <a href="https://discord.gg/txzvKMSd">
    <img alt="Discord" src="https://img.shields.io/badge/Discord-Join-5865F2?style=flat-square" />
  </a>
</p>

<!-- QUICK ACTION BUTTONS -->
<p align="center">
  <a href="docs/assembly/README.md">
    <img alt="Build from scratch" src="https://img.shields.io/badge/Build%20from%20scratch-Guide-111111?style=for-the-badge" />
  </a>
  &nbsp;
  <a href="docs/SETUP_GUIDE.md#quick-setup">
    <img alt="Use the model" src="https://img.shields.io/badge/Use%20the%20model-Start%20here-111111?style=for-the-badge" />
  </a>
</p>

<!-- TABLE OF CONTENTS -->
<details>
  <summary><b>Table of Contents</b></summary>
  <ol>
    <li><a href="#see-it-in-action">See It In Action</a></li>
    <li><a href="#about-the-project">About The Project</a></li>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#flight-plan">Flight Plan</a></li>
    <li><a href="#deep-dive-articles">Deep Dive Articles</a></li>
    <li><a href="#documentation">Documentation</a></li>
    <li><a href="#project-structure">Project Structure</a></li>
    <li><a href="#community">Community</a></li>
    <li><a href="#faq">FAQ</a></li>
    <li><a href="#share-pack">Share Pack</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>

<!-- SEE IT IN ACTION -->
## See It In Action

### Assemble the drone and fly it autonomously

<p align="center">
  <b>"Build from scratch" drone holding its position</b>
</p>

<p align="center">
  <a href="https://www.youtube.com/shorts/gf9mxroeurU" target="_blank" rel="noopener noreferrer">
    <img src="https://img.youtube.com/vi/gf9mxroeurU/maxresdefault.jpg" alt="Langostino position hold - click to watch" width="720" loading="lazy" />
  </a>
</p>
<p align="center">
  <a href="https://www.youtube.com/shorts/gf9mxroeurU" target="_blank" rel="noopener noreferrer">
    <img src="https://img.shields.io/badge/Watch%20on-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="Watch on YouTube" />
  </a>
</p>

<p align="center">
  <b>Fly it and improve it!</b>
</p>

<p align="center">
  <a href="https://www.youtube.com/watch?v=hTTbhE_WiQQ" target="_blank" rel="noopener noreferrer">
    <img src="https://img.youtube.com/vi/hTTbhE_WiQQ/maxresdefault.jpg" alt="Fly it and improve it - click to watch" width="720" loading="lazy" />
  </a>
</p>
<p align="center">
  <a href="https://www.youtube.com/watch?v=hTTbhE_WiQQ" target="_blank" rel="noopener noreferrer">
    <img src="https://img.shields.io/badge/Watch%20on-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="Watch on YouTube" />
  </a>
</p>

### Do it yourself, from ZERO!

<p align="center">
  <b>Example of what you will be building</b>
</p>

<p align="center">
  <a href="https://www.youtube.com/watch?v=jgdND4LXlIA" target="_blank" rel="noopener noreferrer">
    <img src="https://img.youtube.com/vi/jgdND4LXlIA/maxresdefault.jpg" alt="Langostino real flight — click to watch" width="720" loading="lazy" />
  </a>
</p>
<p align="center">
  <a href="https://www.youtube.com/watch?v=jgdND4LXlIA" target="_blank" rel="noopener noreferrer">
    <img src="https://img.shields.io/badge/Watch%20on-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="Watch on YouTube" />
  </a>
</p>

<p align="center">
  <sub>If this made you curious, star the repo and share the demo.</sub>
</p>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ABOUT THE PROJECT -->
## About The Project

Langostino is an **open-source reference drone** for real-world autonomous flight — designed to be easy to understand, build, and extend.

* **A practical AI autopilot project** focused on flying in the real world (not a black box)
* **A complete reference platform**: code, docs, and a clear path from "zero" to "first flight"
* **Made to be adapted**: use it as a starting point for your own drone and your own experiments
* **Community-driven**: built in the open, improved by builders and contributors worldwide

### Why It Matters

Autonomous drones shouldn't feel like magic or closed black boxes. Langostino exists to make real-world autonomy **understandable** and **buildable** — so more people can learn, iterate, and ship useful aerial systems.

By sharing a reference drone in the open, we lower the barrier from "curious" to "first flight", and turn progress into something the whole community can reuse.

### Built With

[![ROS](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/) [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/) [![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white)](https://www.raspberrypi.org/) [![INAV](https://img.shields.io/badge/INAV-1976D2?style=for-the-badge)](https://github.com/iNavFlight/inav) [![Gym PyBullet Drones](https://img.shields.io/badge/Gym%20PyBullet%20Drones-2C2C2C?style=for-the-badge)](https://github.com/utiasDSL/gym-pybullet-drones) [![Bittensor](https://img.shields.io/badge/Bittensor-7A3EF0?style=for-the-badge&logo=bittensor&logoColor=white)](https://bittensor.com/)


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

> [!CAUTION]
> **⚠️ Langostino is a real flying machine.**
> * Follow local regulations
> * Test in a safe environment responsibly and keep propellers off during bench tests.
> * Drones can cause injury and property damage. 
> * Use prop guards and appropriate failsafes

Pick what you want to do next — you don't need to be an expert to start.

<table>
  <tr>
    <td width="50%" valign="top">
      <h4>I'm just curious</h4>
      <ul>
        <li>Watch the demo (above)</li>
        <li>Star this repo</li>
        <li>Share it with a friend</li>
      </ul>
    </td>
    <td width="50%" valign="top">
      <h4>I want to build it from scratch</h4>
      <p><a href="docs/assembly/README.md"><b>Assembly Guide</b></a></p>
      <p><i>Includes the parts list (BOM) and step-by-step build instructions.</i></p>
    </td>
  </tr>
  <tr>
    <td width="50%" valign="top">
      <h4>I want to use the model</h4>
      <p><a href="docs/SETUP_GUIDE.md#quick-setup"><b>Setup Guide → Quick Setup</b></a></p>
    </td>
    <td width="50%" valign="top">
      <h4>I want to contribute</h4>
      <ul>
        <li>Join <a href="https://discord.gg/txzvKMSd">Discord</a></li>
        <li>Read <a href="CONTRIBUTING.md">CONTRIBUTING.md</a></li>
        <li>Pick a <a href="https://github.com/swarm-subnet/Langostino/issues?q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22">good first issue</a></li>
      </ul>
    </td>
  </tr>
</table>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- FLIGHT PLAN -->
## Flight Plan

Your path from zero to first flight:

1. **Get the parts** — Use the [BOM](docs/assembly/BOM.md) as your shopping list to source every component.

2. **Assemble the airframe** — Follow the build guide step-by-step. [Build guide →](docs/assembly/README.md)

3. **Install, then wire it up** — Follow the quickstart checklist step by step. [Quickstart →](docs/SETUP_GUIDE.md#quick-setup)

4. **Troubleshooting** — Diagnose setup, hardware, and runtime issues quickly with step-by-step checks. [Troubleshooting guide →](docs/TROUBLESHOOTING_GUIDE.md)

5. **Go outside & share** — Short flight, record a clip, star the repo, post your build. [Join Discord →](https://discord.gg/txzvKMSd)

<p align="center">
  <sub>Want the bigger picture? See the <a href="https://swarm124.com/decentralized-training">Swarm roadmap →</a></sub>
</p>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- DEEP DIVE ARTICLES -->
## Deep Dive Articles

Want to understand the project in depth? Check out our Substack series that walks through every aspect of building and flying Langostino.

| Chapter | Topic | Description |
|---------|-------|-------------|
| [**Chapter 1**][chapter-1-url] | Inside the Drone | Hardware components and drone anatomy |
| [**Chapter 2**][chapter-2-url] | The Wiring Brain | Wiring, connections, and power distribution |
| [**Chapter 3**][chapter-3-url] | From Data to Motion | Software architecture and data flow |
| [**Chapter 3.5**][chapter-3.5-url] | Additional Configurations | Advanced configuration and tuning |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- DOCUMENTATION -->
## Documentation

Find detailed guides for every aspect of the project.

| Category | Document | Description |
|----------|----------|-------------|
| **Building** | [Assembly Guide](docs/assembly/README.md) | Complete build guide from parts to first flight |
| | [Bill of Materials](docs/assembly/BOM.md) | Full parts list with purchase links |
| **INAV** | [INAV Configuration](docs/INAV_GUIDE.md) | Flight controller setup, PID tuning, MSP configuration |
| **ROS2** | [Setup Guide](docs/SETUP_GUIDE.md) | Ubuntu + ROS2 Humble installation on Raspberry Pi |
| | [Commands Guide](docs/COMMANDS_GUIDE.md) | ROS2 commands and node operations |
| | [Config Parameters](docs/CONFIG_PARAMS_GUIDE.md) | Complete ROS2 node configuration reference |
| **Tools** | [Map Proxy](mapproxy/README.md) | Offline map tile server for ground station |
| **Help** | [Troubleshooting](docs/TROUBLESHOOTING_GUIDE.md) | Common issues and solutions |
| | [Contributing](CONTRIBUTING.md) | How to contribute to the project |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- PROJECT STRUCTURE -->
## Project Structure

<details open>
  <summary><b>Click to expand</b></summary>

```
swarm-ros/
├── src/
│   └── swarm_ai_integration/       # Main ROS2 package
│       ├── swarm_ai_integration/   # Python nodes (FC adapter, LiDAR, AI)
│       ├── config/                 # ROS2 parameter files
│       ├── launch/                 # Launch files
│       └── scripts/                # Utility scripts
├── scripts/
│   ├── setup_22_04.sh              # Ubuntu 22.04 setup script
│   ├── setup_24_04.sh              # Ubuntu 24.04 setup script
│   ├── network_setup_*.sh          # Network configuration scripts
│   ├── verify_setup.sh             # Installation verification
│   └── launch.sh                   # Quick launch script
├── docs/
│   ├── assembly/                   # Build guides and BOM
│   ├── SETUP_GUIDE.md              # ROS2 environment setup
│   ├── INAV_GUIDE.md               # Flight controller configuration
│   ├── COMMANDS_GUIDE.md           # ROS2 commands reference
│   ├── CONFIG_PARAMS_GUIDE.md      # Configuration parameters
│   └── TROUBLESHOOTING_GUIDE.md    # Common issues and fixes
├── assets/
│   └── 3D_print_STL/               # 3D printable parts (mounts, etc.)
├── inav-custom-firmware/           # Custom INAV firmware builds
├── mapproxy/                       # Offline map tile server
├── model/                          # AI model files
├── flight-logs/                    # Flight data logs
└── README.md                       # This file
```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- COMMUNITY -->
## Community

Langostino is built with a **global** community of builders and contributors. If you're building, testing, or improving it — we want to hear from you.

<p align="center">
  <a href="https://discord.gg/txzvKMSd">
    <img alt="Join Discord" src="https://img.shields.io/badge/Join%20Discord-5865F2?style=for-the-badge&logo=discord&logoColor=white" />
  </a>
</p>

<p align="center">
  <a href="https://discord.gg/txzvKMSd"><b>Discord</b></a>
  &nbsp;•&nbsp;
  <a href="https://x.com/SwarmSubnet"><b>X</b></a>
  &nbsp;•&nbsp;
  <a href="https://github.com/swarm-subnet"><b>GitHub</b></a>
</p>

<p align="center">
  <sub>Share your build log or flight clip in Discord — we feature the best builds.</sub>
</p>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- FAQ -->
## FAQ

<details>
  <summary><b>Do I need to be an engineer?</b></summary>
  <p>No. Langostino is designed to be approachable: clear docs, a guided setup, and a community that can help you get started.</p>
</details>

<details>
  <summary><b>How do I get started?</b></summary>
  <p>Start with the <a href="docs/SETUP_GUIDE.md#quick-setup">Quick Setup Guide</a> if you already have compatible hardware. Otherwise, refer to the <a href="docs/assembly/README.md">Assembly Guide</a> for a list of materials and steps to follow. Join <a href="https://discord.gg/txzvKMSd">Discord</a> if you have any questions.</p>
</details>

<details>
  <summary><b>How do I contribute?</b></summary>
  <p>Join <a href="https://discord.gg/txzvKMSd">Discord</a>, check <a href="CONTRIBUTING.md">CONTRIBUTING.md</a>, and pick a <a href="https://github.com/swarm-subnet/Langostino/issues?q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22">good first issue</a>.</p>
</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- SHARE PACK -->
## Share Pack

Want to help Langostino reach more builders? Copy/paste one of these.

<details>
  <summary><b>X (Twitter)</b></summary>

  ```
  Open-source AI autopilot for real-world drones

  Langostino is a reference drone you can understand, build, and fly — built with a global community.

  https://github.com/swarm-subnet/Langostino
  ```
</details>

<details>
  <summary><b>Where to share</b></summary>

  * **Hackaday** — open hardware / DIY audience
  * **DIY Drones** — builders who actually fly
  * **Reddit** — consider: r/drones, r/Multicopter, r/robotics (follow each subreddit's rules)
</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See [LICENSE](LICENSE.md) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

<p align="center">
  <b>Happy flying!</b>
</p>

<!-- MARKDOWN LINKS -->
[chapter-1-url]: https://substack.com/home/post/p-175604069
[chapter-2-url]: https://substack.com/home/post/p-176136139
[chapter-3-url]: https://substack.com/home/post/p-177453660
[chapter-3.5-url]: https://substack.com/home/post/p-180586067
