import os

# === Settings ===
search_dir = r"C:\Users\Pyka\AppData\Local\Arduino15\packages\MegaCoreX"
keyword = "BOD_LVL_BODLEVEL2_gc"   # ← change this to whatever keyword you need

# === Search ===
for root, dirs, files in os.walk(search_dir):
    for file in files:
        file_path = os.path.join(root, file)
        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                for line_num, line in enumerate(f, start=1):
                    if keyword in line:
                        print(f"{file_path} (line {line_num})")
                        break  # stop after first match per file
        except Exception as e:
            print(f"Could not read {file_path}: {e}")
