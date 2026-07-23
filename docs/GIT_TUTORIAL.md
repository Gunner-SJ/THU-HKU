# Git Pull & Push（拉取与上传）

Repo: `git@github.com:Gunner-SJ/THU-HKU.git`

---

## Clone — first-time download of the whole repository

```bash
git clone git@github.com:Gunner-SJ/THU-HKU.git
cd THU-HKU
```

---

## Pull — download latest updates from GitHub

```bash
cd ~/ws_ros2    # or: cd ~/THU-HKU
git pull
```

---

## Push — upload your local commits to GitHub

Check what changed:

```bash
git status
```

Stage files:

```bash
git add .
```

Save a local snapshot:

```bash
git commit -m "Short reason for the change."
```

Upload to GitHub:

```bash
git push
```

---

## SSH test — verify GitHub authentication

```bash
ssh -T git@github.com
```
