# Instructions for Setting Up a Dev Container

![image](./img/vscode-devcontainer.png)

-	If on Windows: first install WSL2: https://learn.microsoft.com/en-us/windows/wsl/install 

### Docker
-	Download and install either Docker Engine or Docker Desktop
    -	Windows: Download and install Docker Desktop: https://docs.docker.com/desktop/install/windows-install/ 
    -	Linux: https://docs.docker.com/engine/install/ubuntu/ 
        -	Docker Engine (Terminal based; Prefered over Docker Desktop for Ubuntu)
-	(if using GPU, e.g., for deep learning) Download and install NVIDIA Container Toolkit 

### VS Code
-	Windows/Linux: Download and install VS Code: https://code.visualstudio.com/download
-	Go to VS Code extensions panel (CTRL+Shift+X) and install Dev Containers 
-	Install the "Remote Development" extension for Visual Studio Code
-	In VS Code, create a folder for your project, go to the folder, and then open the configuration screen (Ctrl+Shift+P) and type/select “Dev Containers: New Dev Container”
    -	Follow the steps and select appropriate configurations, e.g., select C++ project, CMake version 3.22, and Ubuntu 22.04

### OpenCV
For installing OpenCV within container:
-	Modify the Dockerfile in the .devcontainer by adding the commands from https://medium.com/@albertqueralto/installing-opencv-within-docker-containers-for-computer-vision-and-development-a93b46996520 
-	Note when you are building the image for the first time, it will take a long time (15 minutes)

### GUI display:
#### Linux:
-	A very important step before running the devcontainer in VS Code is to type in the terminal “xhost +”. This allows you to connect to the X11 display server so you can display any GUI windows.
-	Here's that code snippet to convince the X-server to forward on Linux (goes in devcontainer.json)
    ```
    "containerEnv": {
            "DISPLAY": "unix:0"
        },
        "mounts": [
            "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
        ],
        "runArgs": ["--privileged"]
    ```
####  Windows:
-	Download the vcxsrv installer and run leaving everything at default: https://sourceforge.net/projects/vcxsrv/ 
-	Once installed, run XLaunch in Windows, leaving everything as default.
    -	NOTE: use “One large window” when starting XLaunch if your GUI is cropped
-	In .devcontainer>Dockerfile, add  
    ```
	ENV DISPLAY=host.docker.internal:0
    ```
    and then build the container
-	Everything should be set & working now!



## Build Issues:

### Troubleshooting Docker Build Issues 

Over the past, some students encountered issues building the Docker containers, particularly related to the OpenCV and Open3D library installations during the build process. These problems seem more common on older computers/laptops with limited system resources (RAM, CPU).

While the `Dockerfile` we provide builds correctly on newer hardware, this guide offers steps to troubleshoot and potentially resolve build failures on your machine.


### Steps to Resolve Build Issues:

1.  **Start Completely Fresh**
    * Remove **all** existing Docker images, containers, and volumes. You can often do this easily within Docker Desktop by clicking the "trash" icon next to the respective items in the Images, Containers, or Volumes sections.
    * If you have modified **any** files from the original repository (especially in the `.devcontainer` folder), **clone the repository again** from the source to ensure you have the pristine, tested versions.
    * **Warning:** If you make custom changes to files within the `.devcontainer` folder (like `Dockerfile` or `devcontainer.json`), you are responsible for debugging any resulting issues. Stick to the original files provided for the best chance of success with these instructions.

2.  **Address Resource Limits (Common `EOF` Error)**
    * Building the `Open3D` library from source code (as done in the `Dockerfile`) is very resource-intensive, requiring significant RAM and CPU power.
    * If your Docker build fails with an error message similar to this:
        ```
        ERROR: failed to receive status: rpc error: code = Unavailable desc = error reading from server: EOF
        ```
        ...it strongly indicates that the Docker build process ran out of allocated system resources (usually RAM).
    * If you encounter this error, try the following steps (you might need to implement several of them):

        * **Increase Swap/Virtual Memory:** Ensure your operating system has ample swap space (virtual memory) configured. While the exact amount needed varies, having a larger buffer can help during peak build demands. Consider adding significant swap space if possible.
            * *Ubuntu:* [How To Add Swap Space on Ubuntu 22.04](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04), [Stack Overflow: Assign More Memory to Docker Container](https://stackoverflow.com/questions/44533319/how-to-assign-more-memory-to-docker-container) (Related discussion)
            * *Windows:* [Increase Virtual Memory (Page file size)](https://support.esri.com/en-us/knowledge-base/increase-virtual-memory-beyond-the-recommended-maximum--000011346)

        * **Allocate More Resources to Docker Desktop:** Follow the updated DevContainer setup instructions provided [in the section below](#Configuring-WSL-2-Resources-for-Docker-Desktop) on how to increase the **RAM and CPU cores** allocated to Docker Desktop. (If using Windows with WSL2, this involves editing the `.wslconfig` file). **This is a very important step**, as the default resource limits, especially on older or resource-constrained systems, are often insufficient for this build.

        * **Reduce Build Parallelism:** Edit the `Dockerfile`. Find the line in the Open3D installation section that compiles the code, which looks like `make -j$(nproc)`. This command tries to use all available CPU cores, which can overwhelm limited RAM. Change it to use a fixed, smaller number of cores. This will make the build *slower* but less resource-intensive. Start by changing it to:
            ```dockerfile
            # Change this line in the Open3D build section of your Dockerfile
            make -j4
            ```
            If `make -j4` still fails due to resources, try `make -j2` or even just `make` (which uses a single core).

        * **Check Available Disk Space:** Ensure the hard drive where Docker stores its images, build cache, and volumes has plenty of free space. Building large libraries requires significant disk space (tens of Gigabytes recommended). A full or nearly full drive will cause build failures.

        * **Remove Unnecessary Libraries:** Say the provided `Dockerfile` also downloads and builds the `OpenCV` library from source. While useful for many tasks, `OpenCV` might not be strictly required for your project. Building it consumes significant time and resources. To potentially free up resources, you can **comment out or delete** the sections in the `Dockerfile` related to `OpenCV`. Look for the `WORKDIR /opencv` line and the subsequent `RUN` commands that use `wget`, `unzip`, `cmake`, and `make` specifically for OpenCV.

3.  **Try an Incremental Build**
    * If the full build continues to fail even after addressing resources, try building the image stage by stage.
    * Open the `Dockerfile` in a text editor.
    * Comment out (by adding a `#` at the beginning of the line) all the `RUN` commands *after* the initial package installations (e.g., comment out the OpenCV build section and the Open3D build section).
    * Try building this basic image using the VS Code Dev Containers extension ("Rebuild Container" or "Reopen in Container").
    * If the basic build succeeds, uncomment the next logical block of `RUN` commands (e.g., the `apt-get install` for Open3D dependencies, or the OpenCV build steps if you kept them).
    * Rebuild the container.
    * Continue this process – uncommenting a section and rebuilding – until you find the specific `RUN` command block that causes the failure. This helps pinpoint where the issue lies (though it's often the resource-intensive `make` command for Open3D).

4.  **Use a More Powerful Computer**
    * If you have tried all the above steps and the build still fails consistently, it might unfortunately be a hardware limitation of your current computer. Building large C++ libraries like Open3D from source code is computationally demanding.
    * If possible, try performing the initial Docker build process on a different, more powerful computer (e.g., a newer desktop/laptop, a university lab computer). Once the image is built successfully, you might be able to use it even on the less powerful machine (though runtime performance could still be a factor).

5.  **Alternative: Use Python and Pre-built Open3D**
    * If getting the C++ Docker environment built remains prohibitively difficult, consider implementing the core algorithm logic directly in Python.
    * You can typically install a pre-compiled version of the `Open3D` library for Python easily using `pip` (Python's package installer). Open a terminal or command prompt with Python installed and run:
        ```bash
        pip install open3d
        ```
    * Say the `Open3D` library is primarily used for loading and displaying point cloud data. If `pip install open3d` works on your system, you can use this Python version to handle the data and focus on coding the required algorithms in Python.
    * You could also explore other Python libraries capable of loading and visualizing point clouds if `Open3D` via pip still presents issues on your system. The specific library for loading/visualization isn't critical as long as you can access the data and implement the necessary processing steps.


Hopefully, these steps help you get your development environment up and running! Remember to start fresh (Step 1) and focus on resource allocation (Step 2) first, as these are the most common culprits for the `EOF` error during the Open3D build.










### Configuring WSL 2 Resources for Docker Desktop

When Docker Desktop uses the WSL 2 backend on Windows, resource limits like memory (RAM) and CPU cores are not configured within Docker Desktop settings directly. Instead, they are controlled by the Windows Subsystem for Linux (WSL) itself via a configuration file.

You need to create or edit a file named `.wslconfig` located in your Windows user profile directory (`C:\Users\<YourUsername>`) to adjust these limits.

Here's how:

1.  **Open your Windows User Profile Folder**
    * Open Windows File Explorer.
    * In the address bar, type `%USERPROFILE%` and press Enter.
    * This will navigate you to your main user folder (e.g., `C:\Users\YourUsername`).

2.  **Create or Edit the `.wslconfig` file**
    * Look for a file named exactly **`.wslconfig`**. Note the leading dot (`.`) and ensure it does *not* have a `.txt` extension.
    * **If the file exists:** Right-click it, select "Open with," and choose "Notepad" or another plain text editor (like VS Code, Notepad++, etc.).
    * **If the file does *not* exist:**
        * Right-click in the empty space within the folder.
        * Select "New" -> "Text Document".
        * Immediately rename the new file to `.wslconfig`.
        * **Important:** Windows might hide the `.txt` extension. Ensure you rename it correctly. You may need to enable "File name extensions" in the File Explorer "View" tab to see and remove the `.txt` part. Confirm any warning Windows gives about changing the extension.
        * Open the new `.wslconfig` file with your text editor.

3.  **Add Configuration Settings**
    * All settings must be placed under a `[wsl2]` section header. If the file is new or empty, add this line first.
    * The primary settings to adjust are `memory` and `processors`.

    * `memory`: Sets the maximum RAM WSL 2 distributions can use. Specify units like `MB` or `GB` (e.g., `8GB`, `16000MB`).
        * **Warning:** Do *not* allocate all your system RAM. Leave a significant amount for Windows itself to run smoothly (recommend leaving at least 4GB-8GB for Windows, depending on your total RAM).
    * `processors`: Sets the maximum number of logical CPU processors WSL 2 can use.
        * Check your total available logical processors in Windows Task Manager (Performance tab -> CPU -> Logical processors).
        * **Warning:** Do *not* allocate all your processors. Leave some cores available for Windows.

    * **Example `.wslconfig` content:**

        ```ini
        [wsl2]
        # Sets the maximum RAM WSL 2 can use to 16 Gigabytes
        memory=16GB

        # Sets the maximum number of processors WSL 2 can use to 8
        processors=8

        # Sets the swap space size (virtual memory) to 32GB. Set to 0 to disable swap.
        # If commented out or omitted, WSL uses a default setting.
        swap=32GB
        
        # --- Optional Settings ---

        # Disables automatic memory reclaim. WSL will hold onto allocated memory longer.
        # Can sometimes improve performance for memory-intensive tasks within WSL,
        # but means less memory is returned to Windows automatically.
        # autoMemoryReclaim=disabled
        ```

    * Adjust the values for `memory`,  `processors`, and `swap` based on your system's hardware and your needs.

4.  **Save and Close**
    * Save the changes to your `.wslconfig` file and close the text editor.

5.  **Shut Down WSL Completely**
    * **This step is absolutely critical!** WSL only reads the `.wslconfig` file when it initially starts its virtual machine environment. Existing sessions won't see the changes.
    * Open **Windows PowerShell** or **Command Prompt** (no administrator privileges needed for this command).
    * Execute the following command:
        ```powershell
        wsl --shutdown
        ```
    * Wait a few seconds for WSL and its distributions to fully stop.

6.  **Restart Docker Desktop**
    * Launch Docker Desktop. It will automatically start its WSL 2 backend, which will now adhere to the new resource limits specified in `.wslconfig`.

7.  **Retry Your Docker Build**
    * Attempt your `docker build` command again for the devcontainer. With more RAM and potentially CPU resources available, the build process (especially heavy compilation like Open3D) should be less likely to fail due to resource exhaustion.

---

**Note:** Monitor your system's overall performance during the Docker build after making these changes. If Windows becomes unresponsive, you may have allocated too many resources to WSL 2 and might need to lower the `memory` or `processors` values in `.wslconfig` (remembering to `wsl --shutdown` again after edits). If the build still fails, consider reducing the parallelism in the Dockerfile's `make` command (e.g., `make -j4`) as a complementary step.
