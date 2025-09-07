# Cara Memperbesar Swap ke 8GB di WSL2 Ubuntu 24

## Status Saat Ini
- Current swap: 2GB (dari /dev/sdb)
- Target: 8GB total swap

## Langkah-langkah

### 1. Check Current Swap
```bash
free -h
swapon --show
```

### 2. Buat Swapfile Tambahan 6GB
```bash
sudo fallocate -l 6G /swapfile_additional
sudo chmod 600 /swapfile_additional
```

### 3. Setup sebagai Swap
```bash
sudo mkswap /swapfile_additional
sudo swapon /swapfile_additional
```

### 4. Verify Total Swap 8GB
```bash
free -h
swapon --show
```

### 5. Buat Permanen di /etc/fstab
```bash
echo '/swapfile_additional none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 6. Verify Konfigurasi
```bash
cat /etc/fstab | grep swap
```

## Hasil Akhir
- Total swap: 8GB (2GB existing + 6GB baru)
- Konfigurasi permanen across reboots
- Siap untuk ROS build tanpa memory issue