param(
    [string]$ProjectFile = "MDK-ARM/VIT6.uvprojx"
)

$ErrorActionPreference = "Stop"

function Convert-ToXmlPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    return ($Path -replace "\\", "/").TrimEnd("/")
}

function Get-Stm32CubeRepoRoot {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Contents
    )

    $repoRootCandidates = New-Object System.Collections.Generic.List[string]
    $defaultRepoRoot = Join-Path $env:USERPROFILE "STM32Cube\Repository"

    if (Test-Path $defaultRepoRoot) {
        $repoRootCandidates.Add((Convert-ToXmlPath $defaultRepoRoot))
    }

    $repoRootPattern = '(?i)[A-Z]:/Users/[^/]+/STM32Cube/Repository'
    foreach ($content in $Contents) {
        $matches = [System.Text.RegularExpressions.Regex]::Matches($content, $repoRootPattern)
        foreach ($match in $matches) {
            $candidate = $match.Value
            if (Test-Path ($candidate -replace "/", "\")) {
                $repoRootCandidates.Add((Convert-ToXmlPath $candidate))
            }
        }
    }

    foreach ($candidate in $repoRootCandidates) {
        return $candidate
    }

    throw "STM32Cube Repository root was not found. Checked USERPROFILE and project-referenced paths."
}

function Remove-DuplicateSemicolonPaths {
    param(
        [Parameter(Mandatory = $true)]
        [string]$XmlText
    )

    $includePathPattern = '(?s)<IncludePath>(.*?)</IncludePath>'

    return [System.Text.RegularExpressions.Regex]::Replace(
        $XmlText,
        $includePathPattern,
        {
            param($match)

            $seen = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
            $paths = New-Object System.Collections.Generic.List[string]

            foreach ($segment in ($match.Groups[1].Value -split ';')) {
                $trimmed = $segment.Trim()
                if ([string]::IsNullOrWhiteSpace($trimmed)) {
                    continue
                }

                if ($seen.Add($trimmed)) {
                    $paths.Add($trimmed)
                }
            }

            return "<IncludePath>$([string]::Join(';', $paths))</IncludePath>"
        }
    )
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedProjectInput = $ProjectFile
if (-not [System.IO.Path]::IsPathRooted($resolvedProjectInput)) {
    $resolvedProjectInput = Join-Path $scriptRoot $resolvedProjectInput
}

if (-not (Test-Path $resolvedProjectInput)) {
    throw "Project file not found: $resolvedProjectInput"
}

$portablePathCm4 = "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F"
$portablePathRvdsCm7 = "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM7/r0p1"
$portablePathGccCm7 = "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1"
$uvprojxLegacyPortPattern = '(?s)\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/RVDS/ARM_CM4F/port\.c</FilePath>.*?</File>'
$uvoptxLegacyPortPattern = '(?s)\s*<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/RVDS/ARM_CM4F/port\.c</PathWithFileName>.*?</File>'
$uvprojxAnyPortPattern = '(?s)\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/.+?/port\.c</FilePath>.*?</File>'
$uvoptxAnyPortPattern = '(?s)\s*<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/.+?/port\.c</PathWithFileName>.*?</File>'
$uvprojxDuplicateGccPattern = '(?s)(\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</FilePath>.*?</File>)\s*(<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</FilePath>.*?</File>)'
$uvoptxDuplicateGccPattern = '(?s)(\s*<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</PathWithFileName>.*?</File>)\s*(<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</PathWithFileName>.*?</File>)'
$projectPath = Resolve-Path $resolvedProjectInput
$projectDir = Split-Path -Parent $projectPath
$projectName = [System.IO.Path]::GetFileNameWithoutExtension($projectPath)
$filesToPatch = @($projectPath)
$uvoptxPath = Join-Path $projectDir ($projectName + ".uvoptx")

if (Test-Path $uvoptxPath) {
    $filesToPatch += (Resolve-Path $uvoptxPath)
}

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$rawContents = @()
foreach ($file in $filesToPatch) {
    $rawContents += (Get-Content -Path $file -Raw)
}
$stm32CubeRepoRoot = Get-Stm32CubeRepoRoot -Contents $rawContents
$repoRootPattern = '(?i)[A-Z]:/Users/[^/]+/STM32Cube/Repository'

foreach ($file in $filesToPatch) {
    $content = Get-Content -Path $file -Raw
    $updated = $content

    if ($file -like "*.uvprojx") {
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvprojxLegacyPortPattern, "", [System.Text.RegularExpressions.RegexOptions]::Singleline)
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvprojxDuplicateGccPattern, '$1', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    }
    if ($file -like "*.uvoptx") {
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvoptxLegacyPortPattern, "", [System.Text.RegularExpressions.RegexOptions]::Singleline)
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvoptxDuplicateGccPattern, '$1', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    }

    $updated = $updated.Replace($portablePathCm4, $portablePathGccCm7)
    $updated = $updated.Replace($portablePathRvdsCm7, $portablePathGccCm7)
    $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $repoRootPattern, $stm32CubeRepoRoot)
    $updated = Remove-DuplicateSemicolonPaths -XmlText $updated

    $duplicatePattern = $null
    if ($file -like "*.uvprojx") {
        $duplicatePattern = $uvprojxAnyPortPattern
    } elseif ($file -like "*.uvoptx") {
        $duplicatePattern = $uvoptxAnyPortPattern
        $matches = [System.Text.RegularExpressions.Regex]::Matches($updated, $duplicatePattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if ($matches.Count -gt 1) {
            for ($i = $matches.Count - 1; $i -ge 1; $i--) {
                $updated = $updated.Remove($matches[$i].Index, $matches[$i].Length)
            }
        }
    }

    if ($updated -eq $content) {
        Write-Host "No FreeRTOS portable path update was needed in $file"
    } else {
        [System.IO.File]::WriteAllText($file, $updated, $utf8NoBom)
        Write-Host "Updated FreeRTOS portable path in $file"
    }

    $verify = Get-Content -Path $file -Raw
    if ($verify.Contains($portablePathCm4)) {
        throw "Patch verification failed: CM4F FreeRTOS portable path is still present in $file"
    }
    if ($verify.Contains("portable/RVDS/ARM_CM4F/port.c")) {
        throw "Patch verification failed: legacy RVDS CM4F port.c entry is still present in $file"
    }
    if ($verify.Contains("Middlewares/Third_Party/FreeRTOS/Source/portable/") -and
        -not $verify.Contains($portablePathGccCm7)) {
        throw "Patch verification failed: GCC ARM_CM7 r0p1 portable path not found in $file"
    }
    $repoRootMatches = [System.Text.RegularExpressions.Regex]::Matches($verify, $repoRootPattern)
    foreach ($match in $repoRootMatches) {
        if ((Convert-ToXmlPath $match.Value) -ine $stm32CubeRepoRoot) {
            throw "Patch verification failed: stale STM32Cube Repository path is still present in $file"
        }
    }
    if ($null -ne $duplicatePattern) {
        $remainingMatches = [System.Text.RegularExpressions.Regex]::Matches($verify, $duplicatePattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if ($remainingMatches.Count -gt 1) {
            throw "Patch verification failed: duplicate FreeRTOS port.c entries are still present in $file"
        }
    }
}

Write-Host "Verification OK: using $portablePathGccCm7"
Write-Host "Verification OK: using STM32Cube Repository root $stm32CubeRepoRoot"
