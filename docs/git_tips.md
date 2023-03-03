# 1.上传大文件失败

```git
$ git push origin main
Enumerating objects: 1427, done.
Counting objects: 100% (1421/1421), done.
Delta compression using up to 12 threads
Compressing objects: 100% (1366/1366), done.
Writing objects: 100% (1409/1409), 209.63 MiB | 3.60 MiB/s, done.
Total 1409 (delta 606), reused 0 (delta 0), pack-reused 0
remote: Resolving deltas: 100% (606/606), completed with 7 local objects.
remote: warning: File motor/docs/电机学（美）乌曼.pdf is 95.47 MB; this is larger than GitHub's recommended maximum file size of 50.00 MB
remote: error: Trace: cde25f72872870ca5742ecf57995df53ba3409ab6a632e5a0e32fa487fd2616b
remote: error: See http://git.io/iEPt8g for more information.
remote: error: File motor/docs/自动控制原理.pdf is 128.70 MB; this exceeds GitHub's file size limit of 100.00 MB
remote: error: GH001: Large files detected. You may want to try Git Large File Storage - https://git-lfs.github.com.
To https://github.com/QLiwei/arm-crotex-m.git
 ! [remote rejected] main -> main (pre-receive hook declined)
error: failed to push some refs to 'https://github.com/QLiwei/arm-crotex-m.git'

```

- 这就是不小心本地提交了大文件，无法提交到github了
- 但是可能本地很有很多次 commit ，没办法回滚，这就需要把这个文件关联的所有commit进行修改了。
- 找到指定的太大的那个文件 `motor/docs/... ` ,然后对这个文件进行清理

解决：

```git
 git filter-branch --force --index-filter "git rm -rf --cached --ignore-unmatch motor/docs/*" --prune-empty --tag-name-filter cat -- --all
 
 
 git pull
 git add 
 git commit 
 git push origin main
```

