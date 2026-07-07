# GitHub Upload Guide

새 구조로 GitHub 내용을 완전히 교체하려면 아래 명령을 사용합니다.

```bash
cd ~/Desktop/f1tenth_github_ready
rm -rf .git
git init
git remote add origin https://github.com/yoyoyoyng/f1tenth-autonomous-driving.git
git add .
git commit -m "Rebuild clean portfolio repository"
git branch -M main
git push -u origin main --force
```

기존 GitHub 기록을 유지하면서 덮어쓰려면, 기존 `.git` 폴더가 있는 상태에서 새 파일을 복사한 뒤 다음만 실행합니다.

```bash
git add .
git commit -m "Rebuild clean portfolio repository"
git push
```
