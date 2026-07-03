/**
 * 从拖放 DataTransfer 递归收集文件（含文件夹），参考 robot_motion_editor
 */

function setRelativePath(file: File, relativePath: string): File {
  try {
    Object.defineProperty(file, 'webkitRelativePath', {
      value: relativePath,
      configurable: true,
    });
  } catch {
    /* 部分环境不可写 */
  }
  return file;
}

async function traverseEntry(
  entry: FileSystemEntry,
  parentPath: string,
  files: File[],
): Promise<void> {
  if (entry.isFile) {
    const file = await new Promise<File>((resolve, reject) => {
      (entry as FileSystemFileEntry).file(resolve, reject);
    });
    setRelativePath(file, parentPath + entry.name);
    files.push(file);
  } else if (entry.isDirectory) {
    const dirPath = `${parentPath}${entry.name}/`;
    const reader = (entry as FileSystemDirectoryEntry).createReader();
    let batch: FileSystemEntry[];
    do {
      batch = await new Promise<FileSystemEntry[]>((resolve, reject) => {
        reader.readEntries(resolve, reject);
      });
      for (const child of batch) {
        await traverseEntry(child, dirPath, files);
      }
    } while (batch.length > 0);
  }
}

export async function collectFilesFromDataTransfer(
  dataTransfer: DataTransfer,
): Promise<File[]> {
  const files: File[] = [];

  if (dataTransfer?.items?.length) {
    const entries: FileSystemEntry[] = [];
    for (let i = 0; i < dataTransfer.items.length; i++) {
      const item = dataTransfer.items[i];
      if (!item || item.kind !== 'file') continue;
      const entry = item.webkitGetAsEntry?.();
      if (entry) entries.push(entry);
    }
    if (entries.length > 0) {
      await Promise.all(entries.map((entry) => traverseEntry(entry, '', files)));
    }
  }

  if (files.length === 0 && dataTransfer?.files?.length) {
    for (const file of dataTransfer.files) {
      files.push(file);
    }
  }

  return files;
}

export function hasFileTransfer(dataTransfer: DataTransfer): boolean {
  if (!dataTransfer?.types) return false;
  return Array.from(dataTransfer.types).includes('Files');
}
