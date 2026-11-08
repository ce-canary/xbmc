/*
 *  Copyright (C) 2005-2026 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "BlurayStateSerializer.h"
#include "DVDInputStream.h"
#include "threads/CriticalSection.h"
#if defined(HAS_UDFREAD)
#include "filesystem/UDFContext.h"
#endif

#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <queue>
#include <vector>

extern "C"
{
#include <libbluray/bluray.h>
#include <libbluray/bluray-version.h>
#include <libbluray/filesystem.h>
#include <libbluray/keys.h>
#include <libbluray/overlay.h>
#include <libbluray/player_settings.h>
#include "DVDInputStreamFile.h"
}

#define MAX_PLAYLIST_ID 99999
#define MAX_CLIP_ID 99999
#define BD_EVENT_MENU_OVERLAY -1
#define BD_EVENT_MENU_ERROR   -2
#define BD_EVENT_ENC_ERROR    -3

#define HDMV_PID_VIDEO            0x1011
#define HDMV_PID_VIDEO_EL         0x1015
#define HDMV_PID_AUDIO_FIRST      0x1100
#define HDMV_PID_AUDIO_LAST       0x111f
#define HDMV_PID_PG_FIRST         0x1200
#define HDMV_PID_PG_LAST          0x121f
#define HDMV_PID_PG_HDR_FIRST     0x12a0
#define HDMV_PID_PG_HDR_LAST      0x12bf
#define HDMV_PID_IG_FIRST         0x1400
#define HDMV_PID_IG_LAST          0x141f

class CDVDOverlayImage;
class IVideoPlayer;
class CDVDDemux;

class CDVDInputStreamBluray
  : public CDVDInputStream
  , public CDVDInputStream::IDisplayTime
  , public CDVDInputStream::IChapter
  , public CDVDInputStream::IPosTime
  , public CDVDInputStream::IMenus
  , public CDVDInputStream::IExtentionStream
{
public:
  CDVDInputStreamBluray() = delete;
  CDVDInputStreamBluray(IVideoPlayer* player, const CFileItem& fileitem);
  ~CDVDInputStreamBluray() override;
  bool Open() override;
  void Close() override;
  int Read(uint8_t* buf, int buf_size) override;
  int ReadBlocks(uint8_t* buf, int lba, int num_blocks);
  BD_DIR_H* OpenDir(const char* relPath);
  BD_FILE_H* OpenFile(const char* relPath);
  int64_t Seek(int64_t offset, int whence) override;
  void Abort() override;
  bool IsEOF() override;
  int64_t GetLength() override;
  int GetBlockSize() override { return 6144; }
  ENextStream NextStream() override;


  /* IMenus */
  void ActivateButton() override { UserInput(BD_VK_ENTER); }
  void SelectButton(int iButton) override
  {
    if(iButton < 10)
      UserInput((bd_vk_key_e)(BD_VK_0 + iButton));
  }
  int  GetCurrentButton() override { return 0; }
  int  GetTotalButtons() override { return 0; }
  void OnUp() override  { UserInput(BD_VK_UP); }
  void OnDown() override  { UserInput(BD_VK_DOWN); }
  void OnLeft() override { UserInput(BD_VK_LEFT); }
  void OnRight() override { UserInput(BD_VK_RIGHT); }

  /*! \brief Open the Menu
  * \return true if the menu is successfully opened, false otherwise
  */
  bool OnMenu() override;
  bool OnTopMenu() override;
  void OnBack() override
  {
    if(IsInMenu())
      OnMenu();
  }
  void OnNext() override {}
  void OnPrevious() override {}

  /*!
   * \brief Get the supported menu type
   * \return The supported menu type
  */
  MenuType GetSupportedMenuType() override;

  bool IsInMenu() override;
  bool OnMouseMove(const CPoint &point) override  { return MouseMove(point); }
  bool OnMouseClick(const CPoint &point) override { return MouseClick(point); }
  void SkipStill() override;
  bool GetState(std::string& xmlstate) override;
  bool SetState(const std::string& xmlstate) override;
  bool CanSeek() override;


  void UserInput(bd_vk_key_e vk);
  bool MouseMove(const CPoint &point);
  bool MouseClick(const CPoint &point);

  int GetChapter() override;
  int GetChapterCount() override;
  void GetChapterName(std::string& name, int ch = -1) override;
  std::chrono::milliseconds GetChapterPos(int ch) override;
  bool SeekChapter(int ch) override;

  CDVDInputStream::IDisplayTime* GetIDisplayTime() override { return this; }
  int GetTotalTime() override;
  int GetTime() override;

  CDVDInputStream::IPosTime* GetIPosTime() override { return this; }
  bool PosTime(int ms) override;

  void GetStreamInfo(int pid, std::string &language);

  int Get3dSubtitlePlane(uint16_t pid);

  /*!
   * \brief Check whether a stream is the default of the playlist being played, ie. audio stream
   *        number 1 or presentation graphic stream number 1 of the current clip.
   * \param pid The packet identifier of the stream
   * \return True if the stream is the default audio or subtitle stream, false otherwise
   */
  bool IsDefaultStream(int pid) const;

  /*!
   * \brief Select a stream of the current play item in libbluray in navigation mode, so that the
   *        disc program and later play items keep it.
   * \param pid The packet identifier of the stream
   * \param visible Whether the subtitle stream is shown
   * \return True if the play item presents the stream or the read is held, false otherwise
   */
  bool SetActiveAudioStream(int pid);
  bool SetActiveSubtitleStream(int pid, bool visible);

  //! show or hide the presentation graphic stream libbluray last reported, in navigation mode
  void SetSubtitleVisible(bool visible);

  //! whether the disc lets the user change the audio stream, change the presentation graphic
  //! stream or show or hide it, always true outside navigation mode
  bool CanChangeAudioStream() const;
  bool CanChangeSubtitleStream() const;
  bool CanToggleSubtitle() const;

  //! the presentation graphic display flag libbluray last reported
  bool IsPgEnabled() const { return m_pgEnabled; }

  //! whether the audio stream libbluray last reported changed from a stream of the playing clip, as
  //! a pick by the disc program or the user does; not set for a play item that lacks the stream
  bool IsAudioStreamPick() const { return m_audioStreamPick; }

  struct ClipTime
  {
    uint64_t inTime{0};
    uint64_t startTime{0};
  };

  uint64_t GetReadPos() const { return m_readPos; }

  /*!
   * \brief Get the in time and start time (90 kHz) of the clip a byte was read from.
   * \param readPos The position of the byte in the bytes Read returned since Open
   * \return The clip time, or nothing without a clip
   */
  std::optional<ClipTime> GetClipTime(uint64_t readPos) const;

  //! hand the clip start held at a join to the demuxer opened after it, whose first bytes then
  //! name its play item
  void ReleaseClipStart();
  bool IsClipStartHeld() const { return !m_navmode && m_clipStartHeld && !m_clipStart.empty(); }
  //! whether the stream table of the play item the demuxer opened on lists the Dolby Vision
  //! enhancement layer `pid`, true when libbluray does not tell
  bool ListsEnhancementLayer(int pid) const;
  //! the graphics plane of the presentation graphic stream `pid` of the play item the demuxer
  //! opened on, false when the play item does not list the stream or its video format
  bool GetDemuxerGraphicsPlane(int pid, int& width, int& height) const;
  //! the frame rate `rate`/`scale` of video stream `pid` of the play item the demuxer opened on,
  //! false for an interlaced format or a rate the play item does not state
  bool GetDemuxerFrameRate(int pid, int& rate, int& scale) const;
  bool IsTitleEnded() const { return m_endedTitleTime > 0; }
  bool IsTitleEndWait() const { return m_titleEndWait; }
  //! whether a held title end still waits for its last picture to be presented
  bool IsTitleEndHeld() const { return m_titleEndWait && m_titleInfo != nullptr; }

  //! tell the disc program the title time of the last picture read, DVD_NOPTS_VALUE without one,
  //! and the title time of the player clock, DVD_NOPTS_VALUE while the clock is stopped
  void SetLastPictureTime(double time, double clockTime);

  //! whether a seek while the read is held stays in the title being played out
  bool IsTailInTitle() const;

  //! whether a menu action made the disc program leave the item being played out, reset by the call
  bool TakeItemAbandoned();

  //! whether the disc program left the item being played out and the next stream has not opened
  bool IsJumpPending() const { return m_itemAbandoned; }

  //! run the disc program while the player does not read; its events hold the read as a read would
  void PollEvents();

  //! tell the disc program the title time on screen, DVD_NOPTS_VALUE when not known
  void SetPresentedTime(double time);

  //! tell the disc program that everything read is on screen or will not be shown
  void SetReadPresented();

  //! tell the disc program that the item before the last NextStream has played out
  void ReportItemPlayedOut();

  //! forget that the data being played belongs to a position the disc program has left
  void ResetPresentedLeft() { m_presentedLeft = false; }

  //! whether the read returns no data until the next stream
  bool IsReadHeld() const
  {
    return m_navmode && (m_hold == HOLD_HELD || m_hold == HOLD_STILL || m_hold == HOLD_ERROR ||
                         m_aborted || m_titleEndWait);
  }

  //! the clip the item opened by the last NextStream starts with, when its events named it
  const BLURAY_CLIP_INFO* GetNextItemClip() const;

  //! whether the item opened by the last NextStream continues the playlist of the item before it
  bool NextItemContinuesPlaylist() const { return m_nextItemInPlaylist; }

  //! the title time (ms) the item opened by the last NextStream starts at, when a disc seek moved
  //! it within its playlist
  std::optional<int> GetNextItemSeekTime() const { return m_nextItemSeekTime; }

  void OverlayCallback(const BD_OVERLAY * const);
#ifdef HAVE_LIBBLURAY_BDJ
  void OverlayCallbackARGB(const struct bd_argb_overlay_s * const);
#endif

  BLURAY_TITLE_INFO* GetTitleFromState(const std::string& xmlstate);
  BLURAY_TITLE_INFO* GetTitleLongest();
  BLURAY_TITLE_INFO* GetTitleFile(const std::string& name);

  void ProcessEvent();
  CDVDDemux* GetExtentionDemux() override { return m_pMVCDemux; };
  bool HasExtention() override { return m_bMVCPlayback; }
  bool AreEyesFlipped() override { return m_bFlipEyes; }
  void DisableExtention() override;
  bool OpenNextStream() override;

  void SaveCurrentState(const CStreamDetails& details) override;
  UpdateState UpdateItemFromSavedStates(CFileItem& item, double time, bool& closed) override;
  void UpdateStack(CFileItem& item) override;

protected:
  struct SPlane;

  void OverlayFlush(int64_t pts);
  void UpdateMenuRange(bool hdr);
  void UpdatePendingMenuRange(int64_t titleTime);
  void OverlayClose();
  static void OverlayClear(SPlane& plane, int x, int y, int w, int h);
  static void OverlayInit (SPlane& plane, int w, int h);
  bool ProcessItem(int playitem);

  void CheckPresentedLeft();
  void ReportPresented(int64_t titleTime);

  bool OpenMVCDemux(int playItem);
  bool CloseMVCDemux();
  void SeekMVCDemux(int64_t time);

  IVideoPlayer* m_player = nullptr;
  BLURAY* m_bd = nullptr;
  const BLURAY_TITLE* m_title = nullptr;
  BLURAY_TITLE_INFO* m_titleInfo = nullptr;
  uint32_t m_playlist = MAX_PLAYLIST_ID + 1;
  BLURAY_CLIP_INFO* m_clip = nullptr;
  bool m_hdrVideo = false;

  //! The clip information of the play item being played, ie. every stream its m2ts carries (see
  //! GetStreamInfo). Owned, and only valid while m_clip refers to the same play item.
  struct clpi_cl* m_clipInfo = nullptr;
  uint32_t m_angle = 0;
  bool m_menu = false;
  bool m_isInMainMenu = false;
  bool m_hasOverlay = false;
  bool m_navmode = false;
  int m_dispTimeBeforeRead = 0;
  int                 m_nTitles = -1;
  std::string         m_root;

  // MVC related members
  CDVDDemux*          m_pMVCDemux = nullptr;
  CDVDInputStream    *m_pMVCInput = nullptr;
  bool                m_bMVCPlayback = false;
  int                 m_nMVCSubPathIndex = 0;
  BLURAY_CLIP_INFO*   m_nMVCClip = nullptr;
  bool                m_bFlipEyes = false;
  bool                m_bMVCDisabled = false;
  uint64_t            m_clipStartTime = 0;
  std::queue<int>     m_clipQueue;

  typedef std::shared_ptr<CDVDOverlayImage> SOverlay;
  typedef std::list<SOverlay> SOverlays;

  struct SPlane
  {
    SOverlays o;
    std::vector<BD_PG_PALETTE_ENTRY> palette;
    bool drawing = false;
    int w = 0;
    int h = 0;
  };

  SPlane m_planes[2];
  CCriticalSection m_overlayLock;
  enum EHoldState
  {
    HOLD_NONE = 0,
    HOLD_HELD,
    HOLD_DATA,
    HOLD_STILL,
    HOLD_ERROR,
  } m_hold = HOLD_NONE;
  //! Set by Abort() from the thread closing the player. Kept apart from m_hold, which playback
  //! events overwrite
  std::atomic<bool> m_aborted{false};
  //! Units libbluray has failed to read since it last returned data
  unsigned int m_readErrors = 0;
  BD_EVENT m_event;
#ifdef HAVE_LIBBLURAY_BDJ
  struct bd_argb_buffer_s m_argb;
#endif

  private:
    bool OpenStream(CFileItem &item);
    void SetupPlayerSettings();
    void FreeTitleInfo();
    void ReleaseTitle();
    void FreeClipInfo();
    int ReadData(uint8_t* buf, int buf_size);
    void HoldClipStart(const uint8_t* buf, int size);
    void DropClipStartOnJump();
    void PollHeldJoin();
    bool TakeHeldEvent();
    bool ProcessHeldEvents();
    bool TakeMenuJump(bool joinsInLine, bool dataRead);
    bool AbandonsItem(bool menuJump, bool fromStill) const;
    void MarkMenuInput();
    bool IsJoinHeld() const;
    void DropClipStartOnSeek(bool joinHeld);
    const BLURAY_CLIP_INFO* GetClipAt(uint64_t titlePos) const;

    /*!
     * \brief Whether so many reads in a row have failed that the disc is taken to have gone.
     * Only a successful read clears it, so no read is attempted once it is true.
     */
    bool HasGivenUpReading() const;

    /*!
     * \brief Read the clip information of a play item of the playlist being played.
     * \param playItem The index of the play item, as BD_EVENT_PLAYITEM reports it
     */
    void UpdateClipInfo(unsigned int playItem);

    /*!
     * \brief Find the language of a stream in the stream number table of the current play item.
     * \param pid The packet identifier of the stream
     * \param language Filled in with the language of the stream, if it is found
     * \return True if the playlist presents the stream, false otherwise
     */
    bool GetPlaylistStreamLanguage(int pid, std::string& language) const;

    /*!
     * \brief Find the language of a stream in the clip information of the current play item.
     * \param pid The packet identifier of the stream
     * \param language Filled in with the language of the stream, if it is found
     * \return True if the clip carries the stream, false otherwise
     */
    bool GetClipStreamLanguage(int pid, std::string& language) const;
    bool CanSelectStream() const;
    void SelectPendingStreams();
    void OpenClipSpan();
    void ResetTitleEnd();

    std::unique_ptr<CDVDInputStreamFile> m_pstream;
    std::string m_rootPath;

#if defined(HAS_UDFREAD)
    // Keeps a disc image's UDF volume mounted for as long as the disc is open
    std::optional<XFILE::CUDFMount> m_udfMount;
#endif

    /*! Bluray state serializer handler */
    CBlurayStateSerializer m_blurayStateSerializer;

    /* used during bd_open_stream read block*/
    CCriticalSection m_readBlocksLock;

    std::chrono::steady_clock::time_point m_startWatchTime{};
    std::vector<PlaylistInformation> m_playedPlaylists;
    CCriticalSection m_statesLock;

    struct ClipSpan
    {
      uint64_t readPos{0};
      std::optional<ClipTime> time{};
    };

    uint64_t m_readPos{0};
    std::vector<ClipSpan> m_clipSpans;
    bool m_hasMVCExtension{false};
    int m_endedTitleTime{0};
    bool m_titleEndWait{false};
    int64_t m_lastPictureTime{-1};
    int64_t m_presentedTime{-1};
    int64_t m_clockTime{-1};
    bool m_menuInput{false};
    bool m_itemAbandoned{false};
    std::queue<BD_EVENT> m_heldEvents;
    //! libbluray holds the BD-J notifications of the read until their picture is presented
    bool m_notificationsHeld{false};
    //! the menu range of the last play item read with notifications held, from its title time
    std::optional<bool> m_pendingMenuRange;
    int64_t m_pendingMenuRangeStart{0};
    bool m_presentedLeft{false};
    bool m_readSinceReport{false};
    int64_t m_heldDue{-2};
    int64_t m_reportedTime{-1};
    int64_t m_playedItemEnd{-1};
    const BLURAY_CLIP_INFO* m_nextClip{nullptr};
    bool m_playlistLeft{false};
    bool m_nextItemInPlaylist{false};
    bool m_itemSeeked{false};
    std::optional<int> m_nextItemSeekTime;
    std::vector<uint8_t> m_clipStart;
    size_t m_clipStartOffset{0};
    bool m_clipStartHeld{false};
    uint64_t m_clipStartEnd{0};
    const BLURAY_CLIP_INFO* m_heldClip{nullptr};
    const BLURAY_CLIP_INFO* m_demuxerClip{nullptr};
    bool m_demuxerClipPending{false};
    uint32_t m_pgStreamNumber{0};
    uint32_t m_audioStreamNumber{0};
    bool m_audioStreamPick{false};
    uint32_t m_uoMask{0};
    bool m_pgEnabled{false};
    std::optional<int> m_audioPidPending;
    std::optional<int> m_pgPidPending;
    std::optional<bool> m_pgVisiblePending;
};
