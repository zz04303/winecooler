Import("env")
# Python callback
def on_upload(source, target, env):
    firmware_path = str(source[0])
    env.Execute("python ota_updater.py $UPLOADERFLAGS " + firmware_path)

env.Replace(UPLOADCMD=on_upload)