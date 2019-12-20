# Welcome to the Maze Runner's Homepage

This repository contains the codebase for the Maze Runner website.

# Team Members

- Shawn Shacterman
- Zuyong Li
- Yuanrong Han
- Son Tran

# How to develop this website?

## Setting things up

- This website is built by using Jekyll and GitHub Pages.
- Install Ruby
    + `brew install ruby`
    + Add ruby path into your ~/.bashrc file
        * `export PATH=/usr/local/opt/ruby/bin:$PATH`
    + Add ruby gems path into your ~/.bashrc file
        * Check your ruby version: `ruby -v`
        * `export PATH=$HOME/.gem/ruby/X.X.0/bin:$PATH`
        * Replace `X.X` with your ruby version, e.g. 2.6
- Install Bundler and Jekyll
    + `gem install --user-install bundler jekyll`
- Change directory (cd) to your GitHub repository of the page and run:
  `bundle install`
    + This command will install all the ruby gems needed for the site
- Build your site
    + `bundle exec jekyll serve`
- View your site at `http://localhost:4000/maze-runner`

## Making change to the pages

- Open the file associated to the part that you want to make changes
    + e.g. team.md, design.md, etc.
- Add the content using the syntax of Markdown
    + You can learn how to use Markdown from many sources online.
    + Your content should includes some diagrams, images, or videos
    besides words and formulas to make it more interesting and easier to
    understand.
- Add your images and diagrams to the `assets` directory
