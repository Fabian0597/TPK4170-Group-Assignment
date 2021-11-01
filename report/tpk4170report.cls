\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{tpk4170report}[2020/01/16 Report class for the PS group at MTP, NTNU]

\LoadClass[
headings=standardclasses,
a4paper,
12pt,
headsepline=false,
]{scrbook}

% \LoadClass[a4paper, 12pt]{book}

\RequirePackage[english]{isodate}
\isodate

\RequirePackage{lineno}
%% PATCH AMS FOR LINENO
\newcommand*\patchAmsMathEnvironmentForLineno[1]{%
  \expandafter\let\csname old#1\expandafter\endcsname\csname #1\endcsname
  \expandafter\let\csname oldend#1\expandafter\endcsname\csname end#1\endcsname
  \renewenvironment{#1}%
     {\linenomath\csname old#1\endcsname}%
     {\csname oldend#1\endcsname\endlinenomath}}% 
\newcommand*\patchBothAmsMathEnvironmentsForLineno[1]{%
  \patchAmsMathEnvironmentForLineno{#1}%
  \patchAmsMathEnvironmentForLineno{#1*}}%
\AtBeginDocument{%
\patchBothAmsMathEnvironmentsForLineno{equation}%
\patchBothAmsMathEnvironmentsForLineno{align}%
\patchBothAmsMathEnvironmentsForLineno{flalign}%
\patchBothAmsMathEnvironmentsForLineno{alignat}%
\patchBothAmsMathEnvironmentsForLineno{gather}%
\patchBothAmsMathEnvironmentsForLineno{multline}%
}

\RequirePackage{setspace}

\DeclareOption{draft}{
  \linenumbers
  \doublespacing
}

\setkomafont{captionlabel}{\bfseries}
\setcapindent{0em}

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{scrbook}}
\ProcessOptions\relax

\RequirePackage[utf8]{inputenc} 
\RequirePackage{csquotes}
\RequirePackage[english]{babel}
\RequirePackage[T1]{fontenc} 
\RequirePackage{lmodern}

\RequirePackage{amsmath}
\RequirePackage{amsfonts}
\RequirePackage{graphicx}


\RequirePackage[draft=false,automark]{scrlayer-scrpage}
\setkomafont{pageheadfoot}{\normalfont}
% \clearpairofpagestyles
% \lohead{\headmark}
% \rehead{\headmark}
% \rohead{\pagemark}
% \lehead{\pagemark}
% \pagestyle{scrheadings}


\RequirePackage[
backend=biber,
style=ieee,
backref=false,
maxbibnames=99
]{biblatex}

\setkomafont{descriptionlabel}{\bfseries}

\RequirePackage[
inner=25mm,
outer=25mm,
top=25mm,
bottom=35mm
]{geometry}

\RequirePackage{parskip}

% \setlength{\parskip}{0.5em}

\RequirePackage{hyperref}
\hypersetup{
  colorlinks=true, 
  citecolor=blue,
  linkcolor=blue,
  filecolor=blue,
  urlcolor=blue,
}

\raggedbottom